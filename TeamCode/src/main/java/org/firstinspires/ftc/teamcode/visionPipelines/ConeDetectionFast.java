package org.firstinspires.ftc.teamcode.visionPipelines;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.BackCamera;
import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.teamcode.VisionUtils.Cone;
import org.firstinspires.ftc.teamcode.VisionUtils.CameraBasedPosition;
import org.firstinspires.ftc.teamcode.VisionUtils.ConePointMethod;
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
import java.util.Objects;
import java.util.stream.Collectors;

public class ConeDetectionFast extends OpenCvPipeline {
    @Config
    public static class ConeConfig {
        public static boolean updateColors = true;
        public static double distanceCorrectionMult = .7;//.807;
        public static double distanceCorrectionAdd = 0;
        public static double MAX_DISTANCE_ANGLE_CORRECTION = 2;

        public static int coneMinArea = 600;

        public static int RED_MIN_HUE = 161;
        public static int RED_MIN_SATURATION = 80;
        public static int RED_MIN_VALUE = 0;
        public static int RED_MIN_LIGHTNESS = 0;

        public static int RED_MAX_HUE = 200;
        public static int RED_MAX_SATURATION = 255;
        public static int RED_MAX_VALUE = 255;
        public static int RED_MAX_LIGHTNESS= 255;

        public static int BLUE_MIN_HUE = 146;
        public static int BLUE_MIN_SATURATION = 55;
        public static int BLUE_MIN_VALUE = 0;
        public static int BLUE_MIN_LIGHTNESS = 0;

        public static int BLUE_MAX_HUE = 196;
        public static int BLUE_MAX_SATURATION = 255;
        public static int BLUE_MAX_VALUE = 255;
        public static int BLUE_MAX_LIGHTNESS = 255;

        public static double perfectDistance = 13.5;
        public static double perfectTolerance = 2.5;
        public static boolean useVert = false;
        public static String spectrum = "HSV";
    }
    private BackCamera camera;
    public ConePointMethod conePointMethod;
    private static final double CONE_WIDTH = 4;
    private static final double CONE_HEIGHT = 5;
    private static final Scalar BLANK = new Scalar(0,0,0);
    private static final Scalar RED = new Scalar(255,0,0);
    private static final Scalar ORANGE = new Scalar(255,165,0);
    private static final Scalar YELLOW = new Scalar(255,255,0);
    private static final Scalar GREEN = new Scalar(0,255,0);
    private static final Scalar BLUE = new Scalar(0,0,255);
    private static final Scalar PURPLE = new Scalar(255,0,255);
    private static final Scalar BLACK = new Scalar(1,1,1);
    private static final Scalar WHITE = new Scalar(255,255,255);

    private final Mat structuringSmall = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(3,3));
    private final Mat structuringMedium = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(5,5));
    private final Mat structuringLarge = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(7,7));

    private ArrayList<MatOfPoint> rawContours = new ArrayList<>();
    private ArrayList<Cone> unrankedCones = new ArrayList<>();
    private List<Cone> usableCones = new ArrayList<>();
    private List<Cone> rankedCones = new ArrayList<>();
    public volatile Cone conestackGuess = null;
    private Cone tracking;
    private Point camCenter;
    private Team team;
    private Rect tempRect;
    private Point tempPoint;

    private final Mat resizing = new Mat();
    private final Mat undistortion = new Mat();
    private final Mat mask = new Mat();
    private final Mat intrinsic = new Mat(3, 3, CvType.CV_64F, new Scalar(0));
    private final Mat newIntrinsic = new Mat(3, 3, CvType.CV_64F, new Scalar(0));
    private final Mat distortions = new Mat(1, 5, CvType.CV_64F, new Scalar(0));
    private final Mat hierarchy = new Mat();

    Scalar redLower;
    Scalar redUpper;
    Scalar blueLower;
    Scalar blueUpper;

    public ConeDetectionFast(Team team, BackCamera backCamera) {
        this.team = team;
        this.camera = backCamera;
        this.camCenter = getCenter(this.camera.resolution);
        this.conePointMethod = ConePointMethod.MASS;

//        this.intrinsic.put(0, 0, 1425.1540495760205);
//        this.intrinsic.put(0, 1, 0.0);
//        this.intrinsic.put(0, 2, 970.468471085862);
//        this.intrinsic.put(1, 0, 0.0);
//        this.intrinsic.put(1, 1, 1424.155053359882);
//        this.intrinsic.put(1, 2, 501.56296758809754);
//        this.intrinsic.put(2, 0, 0.0);
//        this.intrinsic.put(2, 1, 0.0);
//        this.intrinsic.put(2, 2, 1.0);
//        this.newIntrinsic.put(0, 0, 316.2171630859375);
//        this.newIntrinsic.put(0, 1, 0.0);
//        this.newIntrinsic.put(0, 2, 217.89658556168433);
//        this.newIntrinsic.put(1, 0, 0.0);
//        this.newIntrinsic.put(1, 1, 311.8413391113281);
//        this.newIntrinsic.put(1, 2, 110.56965838274755);
//        this.newIntrinsic.put(2, 0, 0.0);
//        this.newIntrinsic.put(2, 1, 0.0);
//        this.newIntrinsic.put(2, 2, 1.0);
//        this.distortions.put(0, 0, 0.044556637596306355);
//        this.distortions.put(0, 1, -0.1953025456808677);
//        this.distortions.put(0, 2, -0.0013077514011058707);
//        this.distortions.put(0, 3, 2.538055016042503e-05);
//        this.distortions.put(0, 4, 0.15600849135540265);

        this.intrinsic.put(0, 0, 1426.5985779804787);
        this.intrinsic.put(0, 1, 0.0);
        this.intrinsic.put(0, 2, 972.3839433768713);
        this.intrinsic.put(1, 0, 0.0);
        this.intrinsic.put(1, 1, 1425.4201782937196);
        this.intrinsic.put(1, 2, 502.392108240942);
        this.intrinsic.put(2, 0, 0.0);
        this.intrinsic.put(2, 1, 0.0);
        this.intrinsic.put(2, 2, 1.0);
        this.newIntrinsic.put(0, 0, 1409.3309326171875);
        this.newIntrinsic.put(0, 1, 0.0);
        this.newIntrinsic.put(0, 2, 973.1659712699111);
        this.newIntrinsic.put(1, 0, 0.0);
        this.newIntrinsic.put(1, 1, 1408.265869140625);
        this.newIntrinsic.put(1, 2, 499.84253896542214);
        this.newIntrinsic.put(2, 0, 0.0);
        this.newIntrinsic.put(2, 1, 0.0);
        this.newIntrinsic.put(2, 2, 1.0);
        this.distortions.put(0, 0, 0.04305789687257069);
        this.distortions.put(0, 1, -0.1860924497772236);
        this.distortions.put(0, 2, -0.0014156458648092228);
        this.distortions.put(0, 3, 0.0004741244848272146);
        this.distortions.put(0, 4, 0.14543734560152707);

        if (Objects.equals(ConeConfig.spectrum, "HSV")) {
            redLower = new Scalar(ConeConfig.RED_MIN_HUE, ConeConfig.RED_MIN_SATURATION, ConeConfig.RED_MIN_VALUE);
            redUpper = new Scalar(ConeConfig.RED_MAX_HUE, ConeConfig.RED_MAX_SATURATION, ConeConfig.RED_MAX_VALUE);
            blueLower = new Scalar(ConeConfig.BLUE_MIN_HUE, ConeConfig.BLUE_MIN_SATURATION, ConeConfig.BLUE_MIN_VALUE);
            blueUpper = new Scalar(ConeConfig.BLUE_MAX_HUE, ConeConfig.BLUE_MAX_SATURATION, ConeConfig.BLUE_MAX_VALUE);
        }
        else if (Objects.equals(ConeConfig.spectrum, "HLS")) {
            redLower = new Scalar(ConeConfig.RED_MIN_HUE, ConeConfig.RED_MIN_LIGHTNESS, ConeConfig.RED_MIN_SATURATION);
            redUpper = new Scalar(ConeConfig.RED_MAX_HUE, ConeConfig.RED_MAX_LIGHTNESS, ConeConfig.RED_MAX_SATURATION);
            blueLower = new Scalar(ConeConfig.BLUE_MIN_HUE, ConeConfig.BLUE_MIN_LIGHTNESS, ConeConfig.BLUE_MIN_SATURATION);
            blueUpper = new Scalar(ConeConfig.BLUE_MAX_HUE, ConeConfig.BLUE_MAX_LIGHTNESS, ConeConfig.BLUE_MAX_SATURATION);
        }
    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.resize(input, this.resizing, this.camera.nativeResolution);
        Calib3d.undistort(this.resizing, this.undistortion, this.newIntrinsic, this.distortions);
        Imgproc.resize(this.undistortion, input, this.camera.resolution);

        if (ConeConfig.updateColors) {
            if (Objects.equals(ConeConfig.spectrum, "HSV")) {
                redLower = new Scalar(ConeConfig.RED_MIN_HUE, ConeConfig.RED_MIN_SATURATION, ConeConfig.RED_MIN_VALUE);
                redUpper = new Scalar(ConeConfig.RED_MAX_HUE, ConeConfig.RED_MAX_SATURATION, ConeConfig.RED_MAX_VALUE);
                blueLower = new Scalar(ConeConfig.BLUE_MIN_HUE, ConeConfig.BLUE_MIN_SATURATION, ConeConfig.BLUE_MIN_VALUE);
                blueUpper = new Scalar(ConeConfig.BLUE_MAX_HUE, ConeConfig.BLUE_MAX_SATURATION, ConeConfig.BLUE_MAX_VALUE);
            } else if (Objects.equals(ConeConfig.spectrum, "HLS")) {
                redLower = new Scalar(ConeConfig.RED_MIN_HUE, ConeConfig.RED_MIN_LIGHTNESS, ConeConfig.RED_MIN_SATURATION);
                redUpper = new Scalar(ConeConfig.RED_MAX_HUE, ConeConfig.RED_MAX_LIGHTNESS, ConeConfig.RED_MAX_SATURATION);
                blueLower = new Scalar(ConeConfig.BLUE_MIN_HUE, ConeConfig.BLUE_MIN_LIGHTNESS, ConeConfig.BLUE_MIN_SATURATION);
                blueUpper = new Scalar(ConeConfig.BLUE_MAX_HUE, ConeConfig.BLUE_MAX_LIGHTNESS, ConeConfig.BLUE_MAX_SATURATION);
            }
        }

        if (this.team == Team.RED) {
            if (Objects.equals(ConeConfig.spectrum, "HSV")) Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2HSV_FULL);
            else if (Objects.equals(ConeConfig.spectrum, "HLS")) Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2HLS_FULL);
            Core.inRange(input, redLower, redUpper, this.mask);
        } else if (this.team == Team.BLUE) {
            if (Objects.equals(ConeConfig.spectrum, "HSV")) Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV_FULL);
            else if (Objects.equals(ConeConfig.spectrum, "HLS")) Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HLS_FULL);
            Core.inRange(input, blueLower, blueUpper, this.mask);
        } else { return input; }

        Imgproc.morphologyEx(this.mask,this.mask,Imgproc.MORPH_OPEN, this.structuringLarge);
        Imgproc.morphologyEx(this.mask,this.mask,Imgproc.MORPH_CLOSE, this.structuringMedium);
        Imgproc.findContours(this.mask, this.rawContours, this.hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        this.unrankedCones = new ArrayList<>();//this.unrankedCones.clear();

        for (MatOfPoint contour: this.rawContours) {
            this.tempRect = Imgproc.boundingRect(contour);
            if (this.conePointMethod == ConePointMethod.MASS) { this.tempPoint = new Point(Imgproc.boundingRect(contour).x + Imgproc.boundingRect(contour).width *.5,Imgproc.boundingRect(contour).y); }
            else if (this.conePointMethod == ConePointMethod.TOP){this.tempPoint = getTop(contour); }
            if (Imgproc.contourArea(contour) >= ConeConfig.coneMinArea && this.tempRect.height > this.tempRect.width) {
                double dist;
                double angleDistanceCorrection =  ConeConfig.MAX_DISTANCE_ANGLE_CORRECTION * Math.abs(this.getAngle(this.tempPoint) / Math.toRadians(30));
                if (ConeConfig.useVert) { dist = (this.getDistance(this.tempRect.width, CONE_WIDTH) + this.getDistanceVertical(this.tempRect.height, CONE_HEIGHT)) / 2; }
                else { dist = angleDistanceCorrection + (this.getDistance(this.tempRect.width, CONE_WIDTH)); }
                this.unrankedCones.add(new Cone(this.tempRect.size(), new CameraBasedPosition(dist, this.getAngle(this.tempPoint), this.camera.position), this.tempPoint));
            }
        }

        for (MatOfPoint contour: this.rawContours) contour.release();
        this.rawContours = new ArrayList<>(); //this.rawContours.clear();
        this.rank();

        if (this.team == Team.RED) {
            if (Objects.equals(ConeConfig.spectrum, "HSV")) Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2BGR_FULL);
            else if (Objects.equals(ConeConfig.spectrum, "HLS")) Imgproc.cvtColor(input, input, Imgproc.COLOR_HLS2BGR_FULL);
        } else {
            if (Objects.equals(ConeConfig.spectrum, "HSV")) Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2RGB_FULL);
            else if (Objects.equals(ConeConfig.spectrum, "HLS")) Imgproc.cvtColor(input, input, Imgproc.COLOR_HLS2RGB_FULL);
        }

        input.setTo(BLACK, this.mask);

        for (int i=0;i<this.rankedCones.size();i++) {
            Cone cone = this.rankedCones.get(i);
            if (i==0) { markerOutlined(input, cone.top, new Point(0,0), GREEN, Imgproc.MARKER_STAR,10,2); }
            if (i==1) { markerOutlined(input, cone.top, new Point(0,0), ORANGE, Imgproc.MARKER_STAR,10,2); }
            textOutlined(input, String.valueOf(i), cone.top,new Point(-4,-16), .7, WHITE, 2);
        }

        for (Cone cone: this.unrankedCones) {
            textOutlined(input, String.valueOf(new Size(cone.position.dx,cone.position.dy)), cone.top,new Point(0,20),.7, WHITE, 2);
            if (cone.classification == Cone.Classification.CLOSE) {
                markerOutlined(input, cone.top, new Point(0,0), RED, Imgproc.MARKER_TILTED_CROSS,40,3);
                textOutlined(input, "CLOSE", cone.top,new Point(0,20), .7, RED, 2);
            } else if (cone.classification == Cone.Classification.FAR) {
                markerOutlined(input, cone.top, new Point(0,0), RED, Imgproc.MARKER_TILTED_CROSS,40,3);
                textOutlined(input, "FAR", cone.top,new Point(0,20), .7, RED, 2);
            }
        }
        return input;
    }

    private void textOutlined(Mat input, String text, Point point, Point offset, double scale, Scalar color, int thickness) {
        Point pos = new Point(point.x+offset.x, point.y+ offset.y);
        int font = Imgproc.FONT_HERSHEY_COMPLEX;
        Imgproc.putText(input, text, pos, font, scale, BLACK,thickness+2);
        Imgproc.putText(input, text, pos, font, scale, color,thickness);
    }

    private void markerOutlined(Mat input,Point point, Point offset, Scalar color,int marker, int size, int thickness) {
        Point pos = new Point(point.x + offset.x, point.y + offset.y);
        Imgproc.drawMarker(input, pos, BLACK, marker, size, thickness+3);
        Imgproc.drawMarker(input, pos, color, marker, size, thickness);
    }

    private void rank() {
        this.conestackGuess = null;
        if (this.unrankedCones.size() > 0) {
            this.usableCones = this.unrankedCones.stream().filter(cone -> cone.classification == Cone.Classification.GOOD && !cone.deadzoned).collect(Collectors.toList());
            this.rankedCones = this.usableCones.stream().sorted(Comparator.comparing(cone -> cone.score)).collect(Collectors.toList());
            Collections.reverse(this.rankedCones);
            this.conestackGuess = Collections.max(this.unrankedCones, Comparator.comparing(cone -> cone.size.height));
        } else {
            this.rankedCones = new ArrayList<>();//this.rankedCones.clear();
            this.usableCones = new ArrayList<>();//this.usableCones.clear();
        }
    }

    private static Point getTop(MatOfPoint contour) { return Collections.min(contour.toList(), Comparator.comparing(h -> h.y)); }
    private static Point getCenter(Size size) { return new Point(size.width/2, size.height/2); }
    private double getDistance(double width, double realWidth) {
        double occupiedFOV = this.camera.HFOV * (width / this.camera.resolution.width);
        return ConeConfig.distanceCorrectionAdd + ConeConfig.distanceCorrectionMult * ( (realWidth/2)/Math.tan(occupiedFOV/2) + (realWidth/2) );
    }
    private double getDistanceVertical(double height, double realHeight) {
        double occupiedFOV = this.camera.VFOV * (height / this.camera.resolution.height);
        return ConeConfig.distanceCorrectionAdd + ConeConfig.distanceCorrectionMult * ( (realHeight/2)/Math.tan(occupiedFOV/2) + (realHeight/2) );
    }
    private double getAngle(Point point) {
        return this.camera.HFOV * (point.x / this.camera.resolution.width) - this.camera.HFOV /2;
    }
    public void startTracking(Cone coneToTrack){
        this.tracking = coneToTrack;
    }
    public void stopTracking(){
        this.tracking = null;
    }
    public List<Cone> getRankedCones() {
        return this.rankedCones;
    }
}
