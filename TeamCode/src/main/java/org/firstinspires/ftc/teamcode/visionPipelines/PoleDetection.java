package org.firstinspires.ftc.teamcode.visionPipelines;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class PoleDetection extends OpenCvPipeline {
    @Config
    public static class PoleConfig {
        public static double distanceCorrection = .8;
        public static int minArea = 900;
        public static int YELLOW_THRESH = 25;
        public static int YELLOW_MIN_SATURATION = 75;
        public static int YELLOW_MAX_SATURATION = 255;
        public static int YELLOW_MIN_VALUE = 50;
        public static int YELLOW_MAX_VALUE = 200;
    }

    private static final double POLE_WIDTH = 1;
    private static final Scalar BLANK = new Scalar(0,0,0);
    private static final Scalar RED = new Scalar(255,0,0);
    private static final Scalar YELLOW = new Scalar(255,255,0);
    private static final Scalar BLUE = new Scalar(0,0,255);
    private static final Scalar GREEN = new Scalar(0,255,0);
    private static final Scalar PURPLE = new Scalar(255,0,255);
    private static final Scalar WHITE = new Scalar(255,255,255);
    private static final Scalar ORANGE = new Scalar(255,165,0);
    private final Scalar compare = new Scalar(43);
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
    private final ArrayList<Pole> poles = new ArrayList<>();
    private final Mat edgeBin = new Mat();
    private final Mat color = new Mat();
    ArrayList<MatOfPoint> rawContours = new ArrayList<>();
    public volatile Pole bestPole = null;

    public PoleDetection(Cam cam) {
        this.cam = cam;
        this.camCenter = getCenter(this.cam.res);
        this.HUD = new Mat(this.cam.res, CvType.CV_8UC3, new Scalar(0,0,0));
    }
    @Override
    public Mat processFrame(Mat input) {
        if (input.channels() == 4) {
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGBA2RGB);
        }
        Imgproc.cvtColor(input, this.hsv, Imgproc.COLOR_RGB2HSV_FULL);
        Core.extractChannel(this.hsv, this.hue, 0);
        Core.absdiff(this.hue, this.compare, this.deviation);
        Imgproc.threshold(this.deviation, this.color, PoleConfig.YELLOW_THRESH, 255, Imgproc.THRESH_BINARY_INV);
        Core.inRange(this.hsv, new Scalar(0, PoleConfig.YELLOW_MIN_SATURATION, PoleConfig.YELLOW_MIN_VALUE), new Scalar(255, PoleConfig.YELLOW_MAX_SATURATION, PoleConfig.YELLOW_MAX_VALUE), this.edgeBin);


        Core.bitwise_and(this.color, this.edgeBin, this.color);
        Imgproc.morphologyEx(this.color,this.color,Imgproc.MORPH_ERODE, this.structuringElement);
//        Imgproc.morphologyEx(this.color,this.color,Imgproc.MORPH_OPEN, this.structuringElement);
        Imgproc.morphologyEx(this.color,this.color,Imgproc.MORPH_CLOSE, this.structuringElement);
        Imgproc.morphologyEx(this.color,this.color,Imgproc.MORPH_DILATE, this.structuringElement);

        this.detectPoles();
        this.pickPoles();
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
        for (Pole pole: this.poles) {
            Scalar color = GREEN;
            Imgproc.circle(this.HUD,getTop(pole.contour),4,color,-1);
            ArrayList<MatOfPoint> contours = new ArrayList<>();
            contours.add(pole.contour);
            Imgproc.drawContours(this.HUD, contours, -1, color, 2);
            this.markerOutlined(getCenter(pole.contour), color, Imgproc.MARKER_TRIANGLE_UP, 20,2);
            this.textOutlined((int) pole.position.distance + " in", getCenter(pole.contour, new Point(15,-5)), Imgproc.FONT_HERSHEY_SIMPLEX,.5, color, 2);
            this.textOutlined((int) Math.toDegrees(pole.position.angle) + " deg", getCenter(pole.contour, new Point(15,+15)), Imgproc.FONT_HERSHEY_SIMPLEX,.5, color, 2);
        }

        if (this.bestPole != null) {
            this.markerOutlined(getCenter(this.bestPole.contour, new Point(0,-30)), PURPLE, Imgproc.MARKER_STAR, 20, 2);
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
        return PoleConfig.distanceCorrection * ( (realWidth/2)/Math.tan(occupiedFOV/2) + (realWidth/2) );
    }
    private double getAngle(Point point) {
        return this.cam.FOV * (point.x / this.cam.res.width) - this.cam.FOV/2;
    }
    private void detectPoles() {
        this.rawContours.clear();
        Imgproc.findContours(this.color, this.rawContours, this.hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        this.poles.clear();
        for (MatOfPoint contour: this.rawContours) {
            if (Imgproc.contourArea(contour) >= PoleConfig.minArea && Imgproc.boundingRect(contour).height > Imgproc.boundingRect(contour).width) { //&& Imgproc.isContourConvex(contour)
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                RotatedRect rect = Imgproc.minAreaRect(contour2f);
                this.poles.add(new Pole(contour,rect, new VisionBasedPosition(this.getDistance(rect.size.width, POLE_WIDTH), this.getAngle(getTop(contour))), getTop(contour), this.cam.position.getHeading()));
                contour2f.release();
            }
        }
    }
    private void pickPoles() {
        this.bestPole = null;
        if (this.poles.size() < 1) return;
        this.bestPole = Collections.min(this.poles, Comparator.comparing(pole-> pole.rect.size.width));
    }
}
