package org.firstinspires.ftc.teamcode.visionPipelines;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;

import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;


public class OdometryPipe extends OpenCvPipeline {
    // MAIN CAM: c922 Pro

    @Config
    public static class cv {
        public static double region = .333;
        public static double rho = 9;
        public static double dyMult = 1;
        public static double dxMult = 1;

        public static double lHSV1 =21;
        public static double lHSV2 =0;
        public static double lHSV3 =0;

        public static double uHSV1 =50;
        public static double uHSV2 =255;
        public static double uHSV3 =255;

        public static double lLAB1 =0;
        public static double lLAB2 =100;
        public static double lLAB3 =165;

        public static double uLAB1 =255;
        public static double uLAB2 =151;
        public static double uLAB3 =225;

        public static double lYCrCb1 =0;
        public static double lYCrCb2 =135;
        public static double lYCrCb3 =25;

        public static double uYCrCb1 =255;
        public static double uYCrCb2 =175;
        public static double uYCrCb3 =90;
    }

    Scalar lower1;// = new Scalar(cv.lHSV1, cv.lHSV2, cv.lHSV3);
    Scalar lower2;//  = new Scalar(cv.lLAB1, cv.lLAB2, cv.lLAB3);
    Scalar lower3;//  = new Scalar(cv.lYCrCb1, cv.lYCrCb2, cv.lYCrCb3);

    Scalar upper1;// = new Scalar(cv.uHSV1, cv.uHSV2, cv.uHSV3);
    Scalar upper2;// = new Scalar(cv.uLAB1, cv.uLAB2, cv.uLAB3);
    Scalar upper3;// = new Scalar(cv.uYCrCb1, cv.uYCrCb2, cv.uYCrCb3);

    final Size minSize= new Size(20,50);
    final int poleAreaMin = 400;
    final double poleMin = 0;
    final double poleMax = .5;

    ArrayList<MatOfPoint> contours;

    Mat primary; //hsv
    Mat secondary; //LAB
    Mat tertiary; //Ycrcb

    Mat mask1;
    Mat mask2;
    Mat mask3;
    Mat masked;
    Mat hierarchy;

    Mat highContrast;
    Mat singleChannel;
    Mat mergedChannels;
    ArrayList<Mat> splitChannels;

    ArrayList<MonocularPole> rawPoles;
    Cam cam;
    Mat frame;
    Mat undistorted;
    Mat roi;
    Mat out=new Mat();
    double region;
    double rho;
    double dyMult;
    double dxMult;
    double pixPerRad;

    public OdometryPipe(Cam cam) {
        this.cam = cam;
        this.frame=new Mat();
        this.undistorted=new Mat();
        this.roi=new Mat();
        this.rho=cv.rho;
        this.dyMult=cv.dyMult;
        this.dxMult=cv.dxMult;
        this.lower1 = new Scalar(cv.lHSV1, cv.lHSV2, cv.lHSV3);
        this.lower2  = new Scalar(cv.lLAB1, cv.lLAB2, cv.lLAB3);
        this.lower3  = new Scalar(cv.lYCrCb1, cv.lYCrCb2, cv.lYCrCb3);
        this.upper1 = new Scalar(cv.uHSV1, cv.uHSV2, cv.uHSV3);
        this.upper2 = new Scalar(cv.uLAB1, cv.uLAB2, cv.uLAB3);
        this.upper3 = new Scalar(cv.uYCrCb1, cv.uYCrCb2, cv.uYCrCb3);
//        this.updateFilters();

        this.contours = new ArrayList<MatOfPoint>();
        this.rawPoles = new ArrayList<MonocularPole>();
        this.hierarchy = new Mat();
        this.singleChannel=new Mat();
        this.mergedChannels=new Mat();
        this.highContrast=new Mat();
        this.splitChannels=new ArrayList<>(3);
        this.primary=new Mat();
        this.secondary=new Mat();
        this.tertiary=new Mat();
        this.mask1=new Mat();
        this.mask2=new Mat();
        this.mask3=new Mat();
        this.masked=new Mat();
    }

    private void filterPoles() {
        for (int i = 0; i < this.contours.size(); i++) {
            MatOfPoint2f contour2f = new MatOfPoint2f(this.contours.get(i).toArray());
            RotatedRect angledR = Imgproc.minAreaRect(contour2f);
            Size pos = new Size(angledR.center.x - this.cam.res.width/2, angledR.center.y);
            Size size = angledR.size;
            angledR.angle-= Math.round(angledR.angle / 45) * 45;
            if (size.width>size.height) { // make sure orientation is right
                double switchingDim=size.width;
                size.width=size.height;
                size.height=switchingDim;
            }
            double tempArea = size.area();
            double tempPerimeter = Imgproc.arcLength(contour2f, true);
            double tempRatio = tempArea / Math.pow(tempPerimeter, 2);
            Touching touching=new Touching(pos.height + size.height > this.cam.res.height - 5,pos.height - size.height > 5,pos.width - size.width > 5,pos.width + size.width > this.cam.res.width - 5);
            if (tempArea >= poleAreaMin && Math.abs(tempRatio) <= poleMax && Math.abs(tempRatio) >= poleMin && size.width >= minSize.width && size.height >= minSize.height && size.height > 2 * size.width && !touching.horizontal){
                Imgproc.drawContours(this.frame, contours,i, new Scalar(0, 0, 255), -1,1,hierarchy,0,new Point(0,this.cam.res.height * this.region));
                double bearing = Math.toRadians(66) *  pos.width / this.cam.res.width;
                double occupiedFOV = Math.toRadians(66) * size.width / this.cam.res.width;
                double dEstimate = .5/Math.tan(occupiedFOV/2);
                double dxEstimate = Math.copySign(Math.sin(bearing) * dEstimate, pos.width) * this.dxMult;
                double dyEstimate = Math.cos(bearing) * dEstimate * this.dyMult;
                Size poleEstimate= new Size(dxEstimate,dyEstimate);
                this.rawPoles.add(new MonocularPole(pos, size, angledR,this.contours.get(i),poleEstimate));
                Imgproc.putText(this.frame, String.valueOf(new Size(Math.round(dxEstimate),Math.round(dyEstimate))), new Point(angledR.center.x, angledR.center.y), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 100, 0), 2);
            }
        }
        this.cam.detectedPoles=this.rawPoles;
        this.rawPoles.clear();
        this.contours.clear();
    }
    private void upContrast(){
        this.splitChannels.clear();
        Imgproc.cvtColor(this.frame, this.highContrast, Imgproc.COLOR_RGB2YUV);
        Core.split(this.highContrast, this.splitChannels);
        Imgproc.equalizeHist(this.splitChannels.get(0),this.singleChannel);
        splitChannels.set(0, this.singleChannel);
        Core.merge(this.splitChannels,this.mergedChannels);
        Imgproc.cvtColor(this.mergedChannels, this.frame, Imgproc.COLOR_YUV2RGB);
    }
    private void updateFilters() {
        this.lower1 = new Scalar(cv.lHSV1, cv.lHSV2, cv.lHSV3);
        this.lower2  = new Scalar(cv.lLAB1, cv.lLAB2, cv.lLAB3);
        this.lower3  = new Scalar(cv.lYCrCb1, cv.lYCrCb2, cv.lYCrCb3);
        this.upper1 = new Scalar(cv.uHSV1, cv.uHSV2, cv.uHSV3);
        this.upper2 = new Scalar(cv.uLAB1, cv.uLAB2, cv.uLAB3);
        this.upper3 = new Scalar(cv.uYCrCb1, cv.uYCrCb2, cv.uYCrCb3);
        this.region=cv.region;
        this.rho=cv.rho;
        this.dxMult=cv.dxMult;
        this.dyMult=cv.dyMult;
    }
    private void mask() {
        Imgproc.cvtColor(this.frame, this.primary, Imgproc.COLOR_RGB2HSV_FULL);
        Imgproc.cvtColor(this.frame, this.secondary, Imgproc.COLOR_RGB2Lab);
        Imgproc.cvtColor(this.frame, this.tertiary, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(this.primary, this.lower1, this.upper1, this.mask1);
        Core.inRange(this.secondary, this.lower2, this.upper2, this.mask2);
        Core.inRange(this.tertiary, this.lower3, this.upper3, this.mask3);
        Core.bitwise_and(this.mask1, this.mask2, this.masked);
        Core.bitwise_and(this.mask3, this.masked, this.masked);
        this.masked=masked.submat((int) Math.round(this.cam.res.height * this.region),(int) Math.round(this.cam.res.height * this.region * 2),0,(int) this.cam.res.width);
    }
    private void markings() {
        Imgproc.line(this.frame, new Point(cv.rho, this.cam.res.height/2 + 25), new Point(cv.rho, this.cam.res.height/2 -25), new Scalar(0, 255, 100), 2);
        Imgproc.line(this.frame, new Point(this.cam.res.width/2 + this.pixPerRad * Math.toRadians(30), this.cam.res.height/2 + 5), new Point(this.cam.res.width/2 + this.pixPerRad * Math.toRadians(30), this.cam.res.height/2 -5), new Scalar(0, 255, 100), 2);
        Imgproc.line(this.frame, new Point(this.cam.res.width/2 - this.pixPerRad * Math.toRadians(30), this.cam.res.height/2 + 5), new Point(this.cam.res.width/2 - this.pixPerRad * Math.toRadians(30), this.cam.res.height/2 -5), new Scalar(0, 255, 100), 2);
        Imgproc.line(this.frame, new Point(this.cam.res.width/2, this.cam.res.height/2 + 15), new Point(this.cam.res.width/2, this.cam.res.height/2 -15), new Scalar(0, 255, 100), 2);
    }
    @Override
    public Mat processFrame(Mat input) {
        this.updateFilters();
        this.pixPerRad= (this.cam.res.width - this.rho*2) / Math.toRadians(this.cam.FOV);
        this.frame=input;
        input.release();
        Calib3d.undistort(input, this.undistorted,this.cam.newCamMat, this.cam.dists);
        Imgproc.cvtColor(this.undistorted,this.undistorted,Imgproc.COLOR_BGRA2BGR);
        Imgproc.bilateralFilter(this.undistorted, this.frame, 10, 250, 50,Core.BORDER_DEFAULT);
        this.upContrast();
        this.mask();
        this.markings();
        Imgproc.findContours(this.masked, this.contours, this.hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(this.frame,this.contours,-1, new Scalar(255, 0, 0), -1,1,hierarchy,0,new Point(0,this.cam.res.height * this.region));
        this.filterPoles();
        this.frame.copyTo(out);
        return out;
    }

}

