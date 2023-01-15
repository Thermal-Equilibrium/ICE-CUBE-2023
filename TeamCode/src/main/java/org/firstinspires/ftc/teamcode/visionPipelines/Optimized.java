package org.firstinspires.ftc.teamcode.visionPipelines;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.opencv.core.*;
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
        public static double distanceCorrection = .8;
        public static int coneMinArea = 900;

        public static int RED_THRESH = 10;
        public static int BLUE_THRESH = 25;

        public static int RED_MIN_SATURATION = 110;
        public static int BLUE_MIN_SATURATION = 75;

        public static int RED_MAX_SATURATION = 255;
        public static int BLUE_MAX_SATURATION = 255;

        public static int RED_MIN_VALUE = 50;
        public static int BLUE_MIN_VALUE = 50;

        public static int RED_MAX_VALUE = 255;
        public static int BLUE_MAX_VALUE = 200;

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

    private final Scalar compare = new Scalar(171);
    private final Cam cam;
    private Point camCenter;
    private final Mat HUD;
    private final Mat HUDGray = new Mat();
    private final Mat HUDMask = new Mat();
    private final Mat hue = new Mat();
    private final Mat structuringElement = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_ELLIPSE, new Size(7,7));

    private final Mat hsv = new Mat();

    private final Mat hierarchy = new Mat();
    private final Mat out = new Mat();
    private final Mat deviation = new Mat();

    private final ArrayList<Cone> cones = new ArrayList<>();
    private final Mat edgeBin = new Mat();
    private final Mat color = new Mat();

    ArrayList<MatOfPoint> rawContours = new ArrayList<>();
    ArrayList<MatOfPoint> coneContours = new ArrayList<>();

    public volatile Cone theCone = null;

    public Optimized(Cam cam) {
        this.cam = cam;
        this.camCenter = getCenter(this.cam.res);
        this.HUD = new Mat(this.cam.res, CvType.CV_8UC3, new Scalar(0,0,0));
    }
    @Override
    public Mat processFrame(Mat input) {
        this.cones.clear();
        if (input.channels() == 4) {
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGBA2RGB);
        }
        this.cones.clear();
        if (TEAM == Color.RED) {
            Imgproc.cvtColor(input, this.hsv, Imgproc.COLOR_BGR2HSV_FULL);
            Core.extractChannel(this.hsv, this.hue, 0);
            Core.absdiff(this.hue, this.compare, this.deviation);
            Imgproc.threshold(this.deviation, this.color, VisionConfig.RED_THRESH, 255, Imgproc.THRESH_BINARY_INV);
            Core.inRange(this.hsv, new Scalar(0,VisionConfig.RED_MIN_SATURATION,VisionConfig.RED_MIN_VALUE), new Scalar(255,VisionConfig.RED_MAX_SATURATION,VisionConfig.RED_MAX_VALUE), this.edgeBin);
        }
        else if (TEAM == Color.BLUE) {
            Imgproc.cvtColor(input, this.hsv, Imgproc.COLOR_RGB2HSV_FULL);
            Core.extractChannel(this.hsv, this.hue, 0);
            Core.absdiff(this.hue, this.compare, this.deviation);
            Imgproc.threshold(this.deviation, this.color, VisionConfig.BLUE_THRESH, 255, Imgproc.THRESH_BINARY_INV);
            Core.inRange(this.hsv, new Scalar(0,VisionConfig.BLUE_MIN_SATURATION,VisionConfig.BLUE_MIN_VALUE), new Scalar(255,VisionConfig.BLUE_MAX_SATURATION,VisionConfig.BLUE_MAX_VALUE), this.edgeBin);
        }

        Core.bitwise_and(this.color, this.edgeBin, this.color);
        Imgproc.morphologyEx(this.color,this.color,Imgproc.MORPH_ERODE, this.structuringElement);
//        Imgproc.morphologyEx(this.color,this.color,Imgproc.MORPH_OPEN, this.structuringElement);
        Imgproc.morphologyEx(this.color,this.color,Imgproc.MORPH_CLOSE, this.structuringElement);
        Imgproc.morphologyEx(this.color,this.color,Imgproc.MORPH_DILATE, this.structuringElement);

        this.detectCones();
        this.coneClassification();
        this.theCone = this.requestCone(true);
        this.doHUD();
        this.finalize(this.deviation).copyTo(out);
        return out;
    }
    private Mat finalize(Mat input) {
        if (input.channels()==1) {
            Imgproc.cvtColor(this.HUD,this.HUDGray,Imgproc.COLOR_RGB2GRAY);
            Imgproc.threshold(this.HUDGray,this.HUDMask,1, 255, Imgproc.THRESH_BINARY);
            input.setTo(new Scalar(0), this.HUDMask);
            Core.add(input, this.HUDGray, input);
        }
        else if (input.channels()==3) {
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
    private void coneClassification() {
        for (MatOfPoint contour: this.coneContours) {
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
            this.cones.add(new Cone(contour, position, getTop(contour), classification, this.cam.position.getHeading()));
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
