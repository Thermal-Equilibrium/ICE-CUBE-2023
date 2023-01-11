package org.firstinspires.ftc.teamcode.visionPipelines;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.opencv.core.*;
import org.opencv.imgproc.CLAHE;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;


public class Optimized extends OpenCvPipeline {
    @Config
    public static class VisionConfig {
        public static double CAMERA_ANGLE = Math.toRadians(-30);
        public static double distanceCorrection = .8;
        public static int coneMinArea = 900;
        public static int poleMinArea = 1000;
        public static int SL = 180;
        public static int RED_MIN_SATURATION = 100;
        public static int BLUE_MIN_SATURATION = 20;
        public static int YELLOW_MIN_SATURATION = 180;
        public static int MIN_YUMA = 0;
        public static int MAX_YUMA = 100;
        public static int YUMA = 100;
        public static double perfectDistance = 13.5;
        public static double perfectTolerance = 2.5;
    }
    private static final Color TEAM = Color.RED;
    private static final double POLE_WIDTH = 1;
    private static final double CONE_WIDTH = 4;
    private static final double CONE_HEIGHT = 5;
    private static final Scalar BLANK = new Scalar(0,0,0);
    private static final Scalar RED = new Scalar(255,0,0);
    private static final Scalar YELLOW = new Scalar(255,255,0);
    private static final Scalar BLUE = new Scalar(0,0,255);
    private static final Scalar GREEN = new Scalar(0,255,0);
    private static final Scalar ORANGE = new Scalar(255,165,0);
    private static final Scalar PURPLE = new Scalar(255,0,255);
    private static final Scalar WHITE = new Scalar(255,255,255);
    private final Cam cam;
    private Point camCenter;
    private final Mat HUD;
    private final Mat HUDGray = new Mat();
    private final Mat HUDMask = new Mat();
    private final ArrayList<Mat> split = new ArrayList<>();
    private final CLAHE clahe = Imgproc.createCLAHE(.01,new Size(8,8));
    private final Mat hue = new Mat();
    private final Mat coneStructure = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_ELLIPSE, new Size(9,9));
    private final ArrayList<Mat> splitChannels = new ArrayList<>();
    private final Mat singleChannel = new Mat();
    private Rect sampleRedRect;
    private Rect sampleYellowRect;
    private Rect sampleBlueRect;
    private final Mat hsv = new Mat();
    private final Mat saturation = new Mat();

    private final Mat hierarchy = new Mat();
    private final Mat sample = new Mat();
    private final Mat out = new Mat();
    private final Mat color = new Mat();
    private final Mat deviation = new Mat();
    private final Mat gameElement = new Mat();

    private final ArrayList<Cone> cones = new ArrayList<>();
    private final Mat test = new Mat();

    ArrayList<MatOfPoint> rawContours = new ArrayList<>();
    ArrayList<MatOfPoint> coneContours = new ArrayList<>();

    public volatile Cone theCone = null;

    long start;
    long end;
    public Optimized(Cam cam) {
        this.cam = cam;
        this.camCenter = getCenter(this.cam.res);
        this.HUD = new Mat(this.cam.res, CvType.CV_8UC3, new Scalar(0,0,0));

        double sampleSize = this.cam.res.height / 15;
        this.sampleRedRect = new Rect(new Point(0, this.cam.res.height - sampleSize), new Size(sampleSize, sampleSize));
        this.sampleYellowRect = new Rect(new Point(this.cam.res.width / 2 - sampleSize /2, this.cam.res.height - sampleSize), new Size(sampleSize, sampleSize));
        this.sampleBlueRect = new Rect(new Point(this.cam.res.width - sampleSize,this.cam.res.height - sampleSize), new Size(sampleSize, sampleSize));
    }
    //    @Override
//    public void init(Mat input) { }
    @Override
    public Mat processFrame(Mat input) {

        start = System.nanoTime();
        long startFilter = System.nanoTime();
//        Imgproc.blur(input,input,new Size(5,5));
//        Imgproc.medianBlur(input,input,5);
//        this.applyClahe(input).copyTo(input);
//        this.constantYuma(input).copyTo(input);
//        this.normalizeYuma(input, VisionConfig.MIN_YUMA,VisionConfig.MAX_YUMA).copyTo(input);
//        this.blurYuma(input,11).copyTo(input);
        long endFilter = System.nanoTime();

        long startDev = System.nanoTime();
        if (TEAM == Color.RED) {
            Imgproc.cvtColor(input, this.hsv, Imgproc.COLOR_BGR2HSV_FULL);
            Imgproc.cvtColor(input.submat(this.sampleRedRect), this.sample, Imgproc.COLOR_BGR2HSV_FULL);
            input.submat(this.sampleRedRect).copyTo(this.sample);
            this.sample.setTo(new Scalar(255,0,0));
            Imgproc.cvtColor(this.sample, this.sample, Imgproc.COLOR_BGR2HSV_FULL);

            Core.extractChannel(this.hsv, this.hue, 0);
            Core.absdiff(this.hue, new Scalar(Core.mean(this.sample).val[0]), this.deviation);

//            Core.extractChannel(this.hsv, this.saturation, 1);
//            Imgproc.threshold(this.saturation, this.saturation, 25, 255, Imgproc.THRESH_BINARY_INV);
        }
        else if (TEAM == Color.BLUE) {
            Imgproc.cvtColor(input, this.hsv, Imgproc.COLOR_RGB2HSV_FULL);
            Imgproc.cvtColor(input.submat(this.sampleBlueRect), this.sample, Imgproc.COLOR_RGB2HSV_FULL);
            input.submat(this.sampleBlueRect).copyTo(this.sample);
            this.sample.setTo(new Scalar(0,0,255));
            Imgproc.cvtColor(this.sample, this.sample, Imgproc.COLOR_RGB2HSV_FULL);

            Core.extractChannel(this.hsv, this.hue, 0);
            Core.absdiff(this.hue, new Scalar(Core.mean(this.sample).val[0]), this.deviation);
//            Imgproc.cvtColor(input, this.hsv, Imgproc.COLOR_RGB2HSV_FULL);
//            Imgproc.cvtColor(input.submat(this.sampleBlueRect), this.sample, Imgproc.COLOR_RGB2HSV_FULL);
//            Core.extractChannel(this.hsv, this.hue, 0);
//            Core.absdiff(this.hue, new Scalar(Core.mean(this.sample).val[0]), this.deviation);
        }
        long endDev = System.nanoTime();
        long startThresh = System.nanoTime();
        Imgproc.threshold(this.deviation, this.color, 25, 255, Imgproc.THRESH_BINARY_INV);
        long endThresh = System.nanoTime();
        long startMorph = System.nanoTime();
        Imgproc.morphologyEx(this.color,this.color,Imgproc.MORPH_ERODE, this.coneStructure);
//        Imgproc.morphologyEx(this.color,this.color,Imgproc.MORPH_OPEN, this.coneStructure);
        Imgproc.morphologyEx(this.color,this.color,Imgproc.MORPH_CLOSE, this.coneStructure);
//        Imgproc.morphologyEx(this.color,this.color,Imgproc.MORPH_DILATE, this.coneStructure);
        long endMorph = System.nanoTime();
        long startContour = System.nanoTime();
        this.cones.clear();
        this.detectCones();
        long endContour = System.nanoTime();
        long startClass = System.nanoTime();
        this.coneClassification(this.coneContours);
        long endClass = System.nanoTime();
        long startHUD = System.nanoTime();
        this.doHUD();
        long endHUD = System.nanoTime();
        long startFinal = System.nanoTime();
        this.finalize(this.deviation).copyTo(out);
        long startReq = System.nanoTime();
        this.theCone = this.requestCone(true);
        long endReq = System.nanoTime();
        long endFinal = System.nanoTime();

        end = System.nanoTime();
        Dashboard.packet.put("filter",(endFilter-startFilter) /1000000);
        Dashboard.packet.put("dev",(endDev-startDev) /1000000);
        Dashboard.packet.put("thresh",(endThresh-startThresh) /1000000);
        Dashboard.packet.put("morph",(endMorph-startMorph) /1000000);
        Dashboard.packet.put("contour",(endContour-startContour) /1000000);
        Dashboard.packet.put("class",(endClass-startClass) /1000000);
        Dashboard.packet.put("HUD",(endHUD-startHUD) /1000000);
        Dashboard.packet.put("final",(endFinal-startFinal) /1000000);
        Dashboard.packet.put("req",(endReq-startReq) /1000000);
        Dashboard.packet.put("total",(end-start) /1000000);
        return out;
    }
    private Mat constantYuma(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YUV);
        Core.split(input, this.splitChannels);
        Core.extractChannel(input,this.singleChannel,0);
        this.singleChannel.setTo(new Scalar(VisionConfig.YUMA));
        this.splitChannels.set(0, this.singleChannel);
        Core.merge(this.splitChannels, input);
        Imgproc.cvtColor(input, input, Imgproc.COLOR_YUV2RGB);
        return input;
    }
    private Mat normalizeYuma(Mat input, int min, int max) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YUV);
        Core.split(input, this.splitChannels);
        Core.extractChannel(input,this.singleChannel,0);
        Core.normalize(this.singleChannel,this.singleChannel,min,max, Core.NORM_MINMAX);
        this.splitChannels.set(0, this.singleChannel);
        Core.merge(this.splitChannels, input);
        Imgproc.cvtColor(input, input, Imgproc.COLOR_YUV2RGB);
        return input;
    }
    private Mat blurYuma(Mat input, int k) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YUV);
        Core.split(input, this.splitChannels);
        Core.extractChannel(input,this.singleChannel,0);
        Imgproc.GaussianBlur(this.singleChannel,this.singleChannel,new Size(k,k),0,0,Core.BORDER_DEFAULT);
//        Imgproc.medianBlur(this.singleChannel,this.singleChannel,k);
        this.splitChannels.set(0, this.singleChannel);
        Core.merge(this.splitChannels, input);
        Imgproc.cvtColor(input, input, Imgproc.COLOR_YUV2RGB);
        return input;
    }
    private Mat applyClahe(Mat input) {
        Imgproc.cvtColor(input,input, Imgproc.COLOR_RGB2Lab);
        Core.split(input, this.split);
        this.clahe.apply(this.split.get(0), this.test);
        this.split.set(0, this.test);
        Core.merge(this.split,input);
        Imgproc.cvtColor(input,input, Imgproc.COLOR_Lab2LRGB);
        this.split.clear();
        return input;
    }
    private Mat finalize(Mat input) {
        if (input.channels()==1) {
            Imgproc.cvtColor(this.HUD,this.HUDGray,Imgproc.COLOR_RGB2GRAY);
            Imgproc.threshold(this.HUDGray,this.HUDMask,1, 255, Imgproc.THRESH_BINARY);
            input.setTo(new Scalar(0), this.HUDMask);
            Core.add(input, this.HUDGray, input);
        }
        else {
            Imgproc.cvtColor(this.HUD,this.HUDGray,Imgproc.COLOR_RGB2GRAY);
            Imgproc.threshold(this.HUDGray,this.HUDMask,1, 255, Imgproc.THRESH_BINARY);
            input.setTo(BLANK, this.HUDMask);
            Core.add(input, this.HUD, input);
        }
        return input;
    }
    private void doHUD() {
        this.HUD.setTo(BLANK);
        Imgproc.drawMarker(this.HUD, this.camCenter, WHITE, Imgproc.MARKER_CROSS);
        this.display();
        Imgproc.rectangle(this.HUD, this.sampleRedRect, new Scalar(2,2,2),3);
        Imgproc.rectangle(this.HUD, this.sampleBlueRect, new Scalar(2,2,2),3);
        Imgproc.rectangle(this.HUD, this.sampleYellowRect, new Scalar(2,2,2),3);
        Imgproc.rectangle(this.HUD, this.sampleRedRect, RED,2);
        Imgproc.rectangle(this.HUD, this.sampleBlueRect, BLUE,2);
        Imgproc.rectangle(this.HUD, this.sampleYellowRect, YELLOW,2);
    }
    private void textOutlined(String text, Point point, int font, double scale, Scalar color, int thickness) {
        Imgproc.putText(this.HUD, text, point, font, scale, new Scalar(2,2,2),thickness+3);
        Imgproc.putText(this.HUD, text, point, font, scale, color,thickness);
    }
    private void markerOutlined(Point point, Scalar color,int marker, int size, int thickness) {
        Imgproc.drawMarker(this.HUD, point, new Scalar(2,2,2), marker, size, thickness+3);
        Imgproc.drawMarker(this.HUD, point, color, marker, size, thickness);
    }
    private void display() {
        for (Cone cone: this.cones) {
            Scalar color;
            if (cone.classification == Cone.Classification.PERFECT) color = GREEN;
            else color = ORANGE;
            Imgproc.circle(this.HUD,getTop(cone.contour),4,color,-1);
            ArrayList<MatOfPoint> contours = new ArrayList<>();
            contours.add(cone.contour);
            Imgproc.drawContours(this.HUD, contours, -1, color, 2);
            this.markerOutlined(getCenter(cone.contour), color, Imgproc.MARKER_TRIANGLE_UP, 20,2);
            this.textOutlined((int) cone.position.distance + " in", getCenter(cone.contour, new Point(15,-5)), Imgproc.FONT_HERSHEY_SIMPLEX,.5, color, 2);
            this.textOutlined((int) Math.toDegrees(cone.position.angle) + " deg", getCenter(cone.contour, new Point(15,+15)), Imgproc.FONT_HERSHEY_SIMPLEX,.5, color, 2);
            this.textOutlined(cone.classification.name(), getCenter(cone.contour, new Point(15,+35)), Imgproc.FONT_HERSHEY_SIMPLEX,.5, color, 2);
        }
        Cone selectionPerfect = requestCone(true);
        if (selectionPerfect != null) {
            this.markerOutlined(getCenter(selectionPerfect.contour, new Point(0,-30)), PURPLE, Imgproc.MARKER_STAR, 20, 2);
        }
        Cone selection = requestCone(false);
        if (selection != null) {
            this.markerOutlined(getCenter(selection.contour, new Point(0,-30)), PURPLE, Imgproc.MARKER_STAR, 20, 2);
        }
    }
    private static Point getTop(MatOfPoint contour) {
        List<Point> points = contour.toList();
        return Collections.min(points, Comparator.comparing(h -> h.y));
    }
    private static Point getCenter(Size rect) {
        return new Point(rect.width/2, rect.height/2);
    }
    private static Point getCenter(MatOfPoint contour) {
        Moments m = Imgproc.moments(contour);
        return new Point((int) (m.get_m10() / m.get_m00()), (int) (m.get_m01() / m.get_m00()));
    }
    private static Point getCenter(MatOfPoint contour, Point offset) {
        Moments m = Imgproc.moments(contour);
        return new Point((int) (m.get_m10() / m.get_m00()) + offset.x, (int) (m.get_m01() / m.get_m00()) + offset.y);
    }
    private double getDistance(double width, double realWidth) {
        double occupiedFOV = this.cam.FOV * (width / this.cam.res.width);
        return VisionConfig.distanceCorrection * ( (realWidth/2)/Math.tan(occupiedFOV/2) + (realWidth/2) );
    }
    private double getAngle(Point point) {
        return this.cam.FOV * (point.x / this.cam.res.width) - this.cam.FOV/2;
    }
    private void detectCones() {
        this.coneContours.clear();
        this.rawContours.clear();
        Imgproc.findContours(this.color, this.rawContours, this.hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint contour: this.rawContours) {
            if (Imgproc.contourArea(contour) >= VisionConfig.coneMinArea && Imgproc.boundingRect(contour).height > Imgproc.boundingRect(contour).width) { //&& Imgproc.isContourConvex(contour)
                this.coneContours.add(contour);
            }
        }
    }
    private void coneClassification(ArrayList<MatOfPoint> contours) {
        for (MatOfPoint contour: contours) {
            Cone.Classification classification;

            VisionBasedPosition position = new VisionBasedPosition( this.getDistance(Imgproc.boundingRect(contour).width,CONE_WIDTH), this.getAngle(getTop(contour)));
            if (position.angle > Math.toRadians(0) && position.angle < Math.toRadians(30)){
                classification = Cone.Classification.DEADZONE;
            }
            else if (position.distance < VisionConfig.perfectDistance - VisionConfig.perfectTolerance) {
                classification = Cone.Classification.CLOSE;
            }
            else if (position.distance > VisionConfig.perfectDistance + VisionConfig.perfectTolerance) {
                classification = Cone.Classification.FAR;
            }
            else {
                classification = Cone.Classification.PERFECT;
            }
            this.cones.add(new Cone(contour, position, getTop(contour), classification));
        }
    }

    private Cone requestCone(boolean requireNonDeadzone) {
        Cone match = null;
        if (this.cones.size() < 1) return null;

        List<Cone> perfect = this.cones.stream().filter(cone -> cone.classification != Cone.Classification.DEADZONE).collect(Collectors.toList());
        if (perfect.size() > 0) match = Collections.min(perfect, Comparator.comparing(cone -> Math.abs(cone.position.distance - VisionConfig.perfectDistance)));
        else if (requireNonDeadzone) return match;
        else {
            match = Collections.min(this.cones, Comparator.comparing(cone -> Math.abs(cone.position.distance - VisionConfig.perfectDistance)));
        }
        return match;
    }


    private ConeStack requestConeStack(boolean requirePerfect) {
        Cone match = null;
        if (this.cones.size() < 1) return null;
        List<Cone> perfect = this.cones.stream().filter(cone -> cone.classification == Cone.Classification.PERFECT).collect(Collectors.toList());
        if (perfect.size() > 0) match = Collections.max(perfect, Comparator.comparing(cone -> cone.contour.height()));
        else if (requirePerfect) return null;
        else {
            match = Collections.max(this.cones, Comparator.comparing(cone -> cone.contour.height()));
        }
        return ConeStack.fromCone(match);
    }
}
