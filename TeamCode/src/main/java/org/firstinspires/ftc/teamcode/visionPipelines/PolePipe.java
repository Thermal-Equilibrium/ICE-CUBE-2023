package org.firstinspires.ftc.teamcode.visionPipelines;
//import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.getCamWidth;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.*;

import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;


public class PolePipe extends OpenCvPipeline {
    static Mat contourMap = new Mat();

    static boolean hasRun = false;
    static double inputWidth;

    static ArrayList<MonocularPole> rawPoles = new ArrayList<MonocularPole>();
    static ArrayList<MatOfPoint> tempContours = new ArrayList<MatOfPoint>();

    private ArrayList<Mat> matsToRelease = new ArrayList<>();

    static Mat highContrast = new Mat();
    static Mat singleChannel = new Mat();
    static Mat mergedChannels = new Mat();

    public static ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();

    public static double getInputWidth() {
        return inputWidth;
    }

    @Config
    public static class cv {
        //        public static double lHSV1 =13;
//        public static double lHSV2 =0;
//        public static double lHSV3 =0;
//
//        public static double uHSV1 =50;
//        public static double uHSV2 =255;
//        public static double uHSV3 =255;
//
//        public static double lLAB1 =0;
//        public static double lLAB2 =99;
//        public static double lLAB3 =149;
//
//        public static double uLAB1 =255;
//        public static double uLAB2 =169;
//        public static double uLAB3 =220;
//
//        public static double lYCrCb1 =0;
//        public static double lYCrCb2 =135;
//        public static double lYCrCb3 =25;
//
//        public static double uYCrCb1 =255;
//        public static double uYCrCb2 =175;
//        public static double uYCrCb3 =101;
        // ^ conservative filter
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
    //    static Scalar lower1;
//    static Scalar lower2;
//    static Scalar lower3;
//
//    static Scalar upper1;
//    static Scalar upper2;
//    static Scalar upper3;
    static Scalar lower1 = new Scalar(cv.lHSV1, cv.lHSV2, cv.lHSV3);
    static Scalar lower2  = new Scalar(cv.lLAB1, cv.lLAB2, cv.lLAB3);
    static Scalar lower3  = new Scalar(cv.lYCrCb1, cv.lYCrCb2, cv.lYCrCb3);

    static Scalar upper1 = new Scalar(cv.uHSV1, cv.uHSV2, cv.uHSV3);
    static Scalar upper2 = new Scalar(cv.uLAB1, cv.uLAB2, cv.uLAB3);
    static Scalar upper3 = new Scalar(cv.uYCrCb1, cv.uYCrCb2, cv.uYCrCb3);

    private static final Mat primary = new Mat(); //hsv
    private static final Mat secondary = new Mat(); //LAB
    private static final Mat tertiary = new Mat(); //Ycrcb

    private static final Mat mask1= new Mat();
    private static final Mat mask2 = new Mat();
    private static final Mat mask3 = new Mat();
    private static final Mat masked = new Mat();
    private static final Mat yellowHierarchy = new Mat();

    private static final ArrayList<Mat> splitChannels=new ArrayList<>(3);

    private static final Size minSize= new Size(20,50);
    private static final int poleAreaMin = 400;
    private static final double poleMin = 0;
    private static final double poleMax = .5;

    static MatOfPoint2f tempContour2f;
    static double tempArea;
    static double tempPerimeter;
    static double tempRatio;
    static Size tempPos;
    static double switchingDim;

    static double angle;
    static RotatedRect angledR;
    static Size size;

    static boolean isDraw=false;
    static ArrayList<MatOfPoint> toDraw = new ArrayList<MatOfPoint>();

    private static ArrayList<MonocularPole> tempRawPoleList = new ArrayList<MonocularPole>();

    public PolePipe() { }

    public static void draw(ArrayList<MatOfPoint> cnts) {
        isDraw=true;
        toDraw=cnts;
    }
    public static ArrayList<MonocularPole> getPoles(){
        return rawPoles;
    }

    static ArrayList<MonocularPole> filterpoles(ArrayList<MatOfPoint> poleContours) {
        tempRawPoleList.clear();
        for (int i = 0; i < poleContours.size(); i++) {
//
//            Moments moment= Imgproc.moments(poleContours.get(i),true);
//            tempPos = new Size((moment.get_m10() / moment.get_m00()) - getCamWidth()/2, moment.get_m01() / moment.get_m00());
//            Rect rect = Imgproc.boundingRect(poleContours.get(i));
//            tempPos = new Size(rect.x - getCamWidth()/2, rect.y);
            tempContour2f = new MatOfPoint2f(poleContours.get(i).toArray());
            angledR = Imgproc.minAreaRect(tempContour2f);
//            tempPos = new Size(angledR.center.x - getCamWidth()/2, angledR.center.y);
            tempPos = new Size(69,69); // fixed the error
            size = angledR.size;
            angle = angledR.angle;
            if (size.width>size.height) { // make sure orientation is right
                switchingDim=size.width;
                size.width=size.height;
                size.height=switchingDim;
            }
            tempArea = size.area();
            tempPerimeter = Imgproc.arcLength(tempContour2f, true);
            tempRatio = tempArea / Math.pow(tempPerimeter, 2);
            if (tempArea >= poleAreaMin && Math.abs(tempRatio) <= poleMax && Math.abs(tempRatio) >= poleMin && size.width >= minSize.width && size.height >= minSize.height && size.height > 2 * size.width){
//                tempRawPoleList.add(new MonocularPole(tempPos, size, tempPerimeter, tempRatio, angle, poleContours.get(i),true, new Touching(false,false,false,false)));
                tempContours.add(poleContours.get(i));
            }
        }
        return tempRawPoleList;
    }

    private static Mat upContrast(Mat theInput){
        splitChannels.clear();
        Imgproc.cvtColor(theInput, highContrast, Imgproc.COLOR_RGB2YUV);
        Core.split(highContrast, splitChannels);
        Imgproc.equalizeHist(splitChannels.get(0),singleChannel);
        splitChannels.set(0, singleChannel);
        Core.merge(splitChannels,mergedChannels);
        Imgproc.cvtColor(mergedChannels, highContrast, Imgproc.COLOR_YUV2RGB);
        return highContrast;
    }

    @Override
    public Mat processFrame(Mat input) {

        lower1 = new Scalar(cv.lHSV1, cv.lHSV2, cv.lHSV3);
        upper1 = new Scalar(cv.uHSV1, cv.uHSV2, cv.uHSV3);

        lower2  = new Scalar(cv.lLAB1, cv.lLAB2, cv.lLAB3);
        upper2 = new Scalar(cv.uLAB1, cv.uLAB2, cv.uLAB3);

        lower3  = new Scalar(cv.lYCrCb1, cv.lYCrCb2, cv.lYCrCb3);
        upper3 = new Scalar(cv.uYCrCb1, cv.uYCrCb2, cv.uYCrCb3);

//        Photo.fastNlMeansDenoisingColored(input,input);
//        Photo.detailEnhance(input,input);
//        Photo.edgePreservingFilter(input,input);
//        Imgproc.Canny();
//        Imgproc.drawContours(contourMap,);
        input = upContrast(input);

//        Imgproc.cvtColor(input, primary, Imgproc.COLOR_BayerRG2BGR_VNG);

        Imgproc.cvtColor(input, primary, Imgproc.COLOR_RGB2HSV_FULL);
        Imgproc.cvtColor(input, secondary, Imgproc.COLOR_RGB2Lab);
        Imgproc.cvtColor(input, tertiary, Imgproc.COLOR_RGB2YCrCb);


        Core.inRange(primary, lower1, upper1, mask1);
        Core.inRange(secondary, lower2, upper2, mask2);
        Core.inRange(tertiary, lower3, upper3, mask3);

        Core.bitwise_and(mask1, mask2, masked);
        Core.bitwise_and(mask3, masked, masked);


        Imgproc.findContours(masked, contours, yellowHierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(input,contours,-1, new Scalar(255, 0, 0), -1);

        rawPoles.clear();
        rawPoles = filterpoles(contours);

        Imgproc.drawContours(input,tempContours,-1, new Scalar(0, 0, 255), -1);

        if (isDraw) {
            Imgproc.drawContours(input,toDraw,-1, new Scalar(0, 255, 0), -1);
            toDraw.clear();
            isDraw=false;
        }

        tempContours.clear();
        contours.clear();
        roundupMemory(primary,secondary,tertiary,mask1,mask2,mask3,masked,yellowHierarchy,highContrast,singleChannel,mergedChannels);
//        roundupMemory(primary,secondary,tertiary,mask1,mask2,mask3,masked,yellowHierarchy,highContrast,singleChannel,mergedChannels);
//        roundupMemory(tertiary,mask3,yellowHierarchy);
        return input;
    }
    public void roundupMemory(Mat... Mats) {
        matsToRelease.clear();
        matsToRelease.addAll(Arrays.asList(Mats));
    }


}

