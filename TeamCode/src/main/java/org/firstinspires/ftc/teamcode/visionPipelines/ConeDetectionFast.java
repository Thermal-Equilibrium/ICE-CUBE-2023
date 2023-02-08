package org.firstinspires.ftc.teamcode.visionPipelines;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.BackCamera;
import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.teamcode.VisionUtils.Cone;
import org.firstinspires.ftc.teamcode.VisionUtils.CameraBasedPosition;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

public class ConeDetectionFast extends OpenCvPipeline {
    @Config
    public static class ConeConfig {
        public static double distanceCorrection = .85;

        public static int coneMinArea = 600;

        public static int RED_MIN_HUE = 161;
        public static int RED_MIN_SATURATION = 80;
        public static int RED_MIN_VALUE = 0;

        public static int RED_MAX_HUE = 200;
        public static int RED_MAX_SATURATION = 255;
        public static int RED_MAX_VALUE = 255;

        public static int BLUE_MIN_HUE = 146;
        public static int BLUE_MIN_SATURATION = 55;
        public static int BLUE_MIN_VALUE = 0;

        public static int BLUE_MAX_HUE = 196;
        public static int BLUE_MAX_SATURATION = 255;
        public static int BLUE_MAX_VALUE = 255;
        public static double perfectDistance = 13.5;
        public static double perfectTolerance = 2.5;
        public static double MAX_ANGLE_DIST = 0;

    }

    private static final double CONE_WIDTH = 4;
    private static final double CONE_HEIGHT = 5;
    private static final Scalar GREEN = new Scalar(0,255,0);
    private static final Scalar PURPLE = new Scalar(255,0,255);
    private static final Scalar ORANGE = new Scalar(255,165,0);
    private Point camCenter;

    private final Mat structuringSmall = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(3,3));
    private final Mat structuringMedium = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(5,5));
    private final Mat structuringLarge = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(7,7));
    private final Mat hierarchy = new Mat();
    private final ArrayList<Cone> cones = new ArrayList<>();
    private final ArrayList<MatOfPoint> rawContours = new ArrayList<>();
    private List<Cone> perfectList;
    private List<Cone> farList;
    private List<Cone> closeList;
    public volatile Cone perfect = null;
    public volatile Cone far = null;
    public volatile Cone close = null;
    public volatile Cone conestackGuess = null;
    private Team team;
    private Rect tempRect;
    private Point tempPoint;
    private Mat mask = new Mat();
    private Mat undistorted;

    private BackCamera camera;

    private Mat camMat;
    private Mat newCamMat;
    private Mat dists;

    private Scalar redLower = new Scalar(ConeConfig.RED_MIN_HUE, ConeConfig.RED_MIN_SATURATION, ConeConfig.RED_MIN_VALUE);
    private Scalar redUpper = new Scalar(ConeConfig.RED_MAX_HUE, ConeConfig.RED_MAX_SATURATION, ConeConfig.RED_MAX_VALUE);
    private Scalar blueLower = new Scalar(ConeConfig.BLUE_MIN_HUE, ConeConfig.BLUE_MIN_SATURATION, ConeConfig.BLUE_MIN_VALUE);
    private Scalar blueUpper = new Scalar(ConeConfig.BLUE_MAX_HUE, ConeConfig.BLUE_MAX_SATURATION, ConeConfig.BLUE_MAX_VALUE);


    public ConeDetectionFast(Team team, BackCamera backCamera) {
        this.team = team;
        this.camera = backCamera;
        this.camCenter = getCenter(this.camera.resolution);
        this.undistorted = new Mat();

        this.camMat = new Mat(3, 3, CvType.CV_64F, new Scalar(0));
        this.camMat.put(0, 0, 1425.1540495760205);
        this.camMat.put(0, 1, 0.0);
        this.camMat.put(0, 2, 970.468471085862);
        this.camMat.put(1, 0, 0.0);
        this.camMat.put(1, 1, 1424.155053359882);
        this.camMat.put(1, 2, 501.56296758809754);
        this.camMat.put(2, 0, 0.0);
        this.camMat.put(2, 1, 0.0);
        this.camMat.put(2, 2, 1.0);

        this.newCamMat = new Mat(3, 3, CvType.CV_64F, new Scalar(0));
        this.newCamMat.put(0, 0, 316.2171630859375);
        this.newCamMat.put(0, 1, 0.0);
        this.newCamMat.put(0, 2, 217.89658556168433);
        this.newCamMat.put(1, 0, 0.0);
        this.newCamMat.put(1, 1, 311.8413391113281);
        this.newCamMat.put(1, 2, 110.56965838274755);
        this.newCamMat.put(2, 0, 0.0);
        this.newCamMat.put(2, 1, 0.0);
        this.newCamMat.put(2, 2, 1.0);

        this.dists = new Mat(1, 5, CvType.CV_64F, new Scalar(0));
        this.dists.put(0, 0, 0.044556637596306355);
        this.dists.put(0, 1, -0.1953025456808677);
        this.dists.put(0, 2, -0.0013077514011058707);
        this.dists.put(0, 3, 2.538055016042503e-05);
        this.dists.put(0, 4, 0.15600849135540265);
    }
    @Override
    public Mat processFrame(Mat input) {
        Calib3d.undistort(input, this.undistorted, this.newCamMat, this.dists);
        this.undistorted.copyTo(input);
        this.redLower = new Scalar(ConeConfig.RED_MIN_HUE, ConeConfig.RED_MIN_SATURATION, ConeConfig.RED_MIN_VALUE);
        this.redUpper = new Scalar(ConeConfig.RED_MAX_HUE, ConeConfig.RED_MAX_SATURATION, ConeConfig.RED_MAX_VALUE);
        this.blueLower = new Scalar(ConeConfig.BLUE_MIN_HUE, ConeConfig.BLUE_MIN_SATURATION, ConeConfig.BLUE_MIN_VALUE);
        this.blueUpper = new Scalar(ConeConfig.BLUE_MAX_HUE, ConeConfig.BLUE_MAX_SATURATION, ConeConfig.BLUE_MAX_VALUE);

        if (this.team == Team.RED) {
            Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2HSV_FULL);
            Core.inRange(input, this.redLower,this.redUpper, this.mask);
        }
        else if (this.team == Team.BLUE) {
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV_FULL);
            Core.inRange(input, this.blueLower, this.blueUpper, this.mask);
        }
        else {
            return input;
        }

        Imgproc.morphologyEx(this.mask,this.mask,Imgproc.MORPH_OPEN, this.structuringLarge);
        Imgproc.morphologyEx(this.mask,this.mask,Imgproc.MORPH_CLOSE, this.structuringMedium);

        Imgproc.findContours(this.mask, this.rawContours, this.hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        this.cones.clear();
        for (MatOfPoint contour: this.rawContours) {
            this.tempRect = Imgproc.boundingRect(contour);
//            this.tempPoint = getTop(contour);
            this.tempPoint = new Point(Imgproc.boundingRect(contour).x + Imgproc.boundingRect(contour).width *.5,Imgproc.boundingRect(contour).y);
            if (Imgproc.contourArea(contour) >= ConeConfig.coneMinArea && this.tempRect.height > this.tempRect.width) {
                double angleDistanceCorrection = ConeConfig.MAX_ANGLE_DIST * Math.abs(Math.toDegrees(this.getAngle(this.tempPoint)))/30;
                this.cones.add(new Cone(this.tempRect.size(), new CameraBasedPosition((this.getDistance(this.tempRect.width,CONE_WIDTH) + this.getDistanceVertical(this.tempRect.height,CONE_HEIGHT))/2 + angleDistanceCorrection, this.getAngle(this.tempPoint), this.camera.position), this.tempPoint));
                Dashboard.packet.put("hor",this.getDistance(this.tempRect.width,CONE_WIDTH));
                Dashboard.packet.put("ver",this.getDistanceVertical(this.tempRect.height,CONE_HEIGHT));

            }
        }
        this.rawContours.clear();

        this.perfect = null;
        this.far = null;
        this.close = null;
        this.conestackGuess = null;
        if (this.cones.size() > 0) {
            this.perfectList = this.cones.stream().filter(cone -> cone.classification == Cone.Classification.PERFECT && !cone.deadzoned).collect(Collectors.toList());
            if (this.perfectList.size() > 0) this.perfect = Collections.min(perfectList, Comparator.comparing(cone -> Math.abs(cone.position.distance - ConeConfig.perfectDistance)));

            this.farList = this.cones.stream().filter(cone -> cone.classification == Cone.Classification.FAR && !cone.deadzoned).collect(Collectors.toList());
            if (this.farList.size() > 0) this.far = Collections.min(this.farList, Comparator.comparing(cone -> Math.abs(cone.position.distance - ConeConfig.perfectDistance)));

            this.closeList = this.cones.stream().filter(cone -> cone.classification == Cone.Classification.CLOSE && !cone.deadzoned).collect(Collectors.toList());
            if (this.closeList.size() > 0) this.close = Collections.min(this.closeList, Comparator.comparing(cone -> Math.abs(cone.position.distance - ConeConfig.perfectDistance)));

            this.conestackGuess = Collections.max(this.cones, Comparator.comparing(cone -> cone.size.height));
        }
        this.cones.clear();
        Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2RGB_FULL);
        input.setTo(new Scalar(0,0,0), this.mask);
        if (this.perfect != null) {
            Imgproc.drawMarker(input, this.perfect.top, GREEN, Imgproc.MARKER_STAR);
            Imgproc.putText(input, String.valueOf(perfect.position.distance), perfect.top, Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(255, 0, 0));
        }
        if (this.far != null) {
            Imgproc.drawMarker(input, this.far.top, PURPLE, Imgproc.MARKER_SQUARE);
            Imgproc.putText(input, String.valueOf(far.position.distance), far.top, Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(255, 0, 0));
        }
        if (this.close != null) {
            Imgproc.drawMarker(input, this.close.top, ORANGE, Imgproc.MARKER_TILTED_CROSS);
            Imgproc.putText(input, String.valueOf(close.position.distance), close.top, Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(255, 0, 0));
        }

        return input;
    }

    private static Point getTop(MatOfPoint contour) {
        return Collections.min(contour.toList(), Comparator.comparing(h -> h.y));
    }
    private static Point getCenter(Size size) {
        return new Point(size.width/2, size.height/2);
    }
    private double getDistance(double width, double realWidth) {
        double occupiedFOV = this.camera.HFOV * (width / this.camera.resolution.width);
        return ConeConfig.distanceCorrection * ( (realWidth/2)/Math.tan(occupiedFOV/2) + (realWidth/2) );
    }
    private double getDistanceVertical(double height, double realHeight) {
        double occupiedFOV = this.camera.VFOV * (height / this.camera.resolution.height);
        return ConeConfig.distanceCorrection * ( (realHeight/2)/Math.tan(occupiedFOV/2) + (realHeight/2) );
    }
    private double getAngle(Point point) {
        return this.camera.HFOV * (point.x / this.camera.resolution.width) - this.camera.HFOV /2;
    }
}
