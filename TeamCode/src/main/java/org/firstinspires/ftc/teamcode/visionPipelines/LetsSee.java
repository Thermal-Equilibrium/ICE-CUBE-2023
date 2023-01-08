package org.firstinspires.ftc.teamcode.visionPipelines;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class LetsSee extends OpenCvPipeline {
    @Config
    public static class VisionConfig {
        public static int coneMinArea = 100;
        public static int poleMinArea = 150;
        public static int SL = 150;
        public static int SU = 255;
        public static int VL = 0;
        public static int VU = 255;
    }
    private Cam cam;

    private Rect sampleRed;
    private Rect sampleYellow;
    private Rect sampleBlue;

    private Mat rgb = new Mat();
    private Mat hsv = new Mat();
    private Mat lab = new Mat();
    private Mat ycrcb = new Mat();

    private Sample red;
    private Sample yellow;
    private Sample blue;

    private Mat out = new Mat();

    private Mat matchYellow = new Mat();

    private Mat yellowDeviation = new Mat();

    private Mat singleChannel = new Mat();

    private Mat gameElement = new Mat();

    private Mat match = new Mat();


    public LetsSee(Cam cam) {
        this.cam = cam;
        this.sampleRed = new Rect(new Point(this.cam.res.width / 2 - this.cam.res.width/2.5, this.cam.res.height - this.cam.res.height / 15), new Size(this.cam.res.height / 15,this.cam.res.height / 15));
        this.sampleYellow = new Rect(new Point(this.cam.res.width / 2 , this.cam.res.height - this.cam.res.height / 15), new Size(this.cam.res.height / 15,this.cam.res.height / 15));
        this.sampleBlue = new Rect(new Point(this.cam.res.width / 2 + this.cam.res.width/2.5,this.cam.res.height - this.cam.res.height / 15), new Size(this.cam.res.height / 15,this.cam.res.height / 15));
    }


    private Mat filter(Mat input) {
        Imgproc.medianBlur(input,input,3);
        return input;
    }

    private void spectrums(Mat input) {
        Imgproc.cvtColor(input, this.rgb, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(input, this.hsv, Imgproc.COLOR_RGB2HSV_FULL);
        Imgproc.cvtColor(input, this.lab, Imgproc.COLOR_RGB2Lab);
        Imgproc.cvtColor(input, this.ycrcb, Imgproc.COLOR_RGB2YCrCb);
    }
    private ArrayList<MatOfPoint> detectCones(Mat input) {
        ArrayList<MatOfPoint> cones = new ArrayList<>();

        //TODO dilate and erode to remove noise

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(input, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint contour: contours) {
            if (Imgproc.contourArea(contour) >= VisionConfig.coneMinArea && Imgproc.isContourConvex(contour)) {
                cones.add(contour);
            }
        }
        return cones;
    }
    private ArrayList<MatOfPoint> detectPoles(Mat input) {
        ArrayList<MatOfPoint> poles = new ArrayList<>();

        //TODO dilate and erode to remove noise

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(input, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint contour: contours) {
            if (Imgproc.contourArea(contour) >= VisionConfig.poleMinArea && Imgproc.isContourConvex(contour)) {
                poles.add(contour);
            }
        }
        return poles;
    }

    @Override
    public void init(Mat input) {

    }

    @Override
    public Mat processFrame(Mat input) {
        filter(input).copyTo(input);
        this.spectrums(input);
        this.red = new Sample(this.rgb.submat(this.sampleRed));
        this.yellow = new Sample(this.rgb.submat(this.sampleYellow));
        this.blue = new Sample(this.rgb.submat(this.sampleBlue));

        Core.absdiff(this.hsv, this.blue.avgHSV, this.yellowDeviation);

        Core.inRange(this.hsv, new Scalar(0, VisionConfig.SL, VisionConfig.VL), new Scalar(255, VisionConfig.SU, VisionConfig.VU), this.gameElement);
        // TODO try ycrcb so i dont have to wrap red
        Core.extractChannel(this.yellowDeviation, this.singleChannel, 0);
        Imgproc.threshold(this.singleChannel, this.matchYellow, 25, 255, Imgproc.THRESH_BINARY_INV);

        Core.bitwise_and(this.matchYellow,this.gameElement,this.match);
        this.yellowDeviation.copyTo(out);
        this.matchYellow.copyTo(out);
        this.singleChannel.copyTo(out);
        this.match.copyTo(out);
//        this.gameElement.copyTo(out);

//        this.rgb.copyTo(out);
        return out;
    }

}
