package org.firstinspires.ftc.teamcode.visionPipelines;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
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
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
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
    public static class ConeDetectionConfig {
        public static boolean updateColors = true;
        public static double distanceCorrectionMult = 1.05;//.807;
        public static double distanceCorrectionAdd = 0;
        public static double MAX_DISTANCE_ANGLE_CORRECTION_ADD = 1;
        public static double MAX_DISTANCE_ANGLE_CORRECTION_ADD_MULT = .13;

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
        public static String spectrum = "HLS";
    }

    private BackCamera camera;
    public ConePointMethod conePointMethod;
    private static final double CONE_WIDTH = 4;
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


    private Point camCenter;
    private Team team;

    private final Mat intrinsic = new Mat(3, 3, CvType.CV_64F, new Scalar(0));
    private final Mat newIntrinsic = new Mat(3, 3, CvType.CV_64F, new Scalar(0));
    private final Mat distortions = new Mat(1, 5, CvType.CV_64F, new Scalar(0));

    Scalar redLower;
    Scalar redUpper;
    Scalar blueLower;
    Scalar blueUpper;

    public volatile Cone conestackGuess = null;
    private volatile List<Cone> cones = new ArrayList<>();

    public ConeDetectionFast(Team team, BackCamera backCamera) {
        this.team = team;
        this.camera = backCamera;
        this.camCenter = getCenter(this.camera.resolution);
        this.conePointMethod = ConePointMethod.MASS;

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

        if (Objects.equals(ConeDetectionConfig.spectrum, "HSV")) {
            redLower = new Scalar(ConeDetectionConfig.RED_MIN_HUE, ConeDetectionConfig.RED_MIN_SATURATION, ConeDetectionConfig.RED_MIN_VALUE);
            redUpper = new Scalar(ConeDetectionConfig.RED_MAX_HUE, ConeDetectionConfig.RED_MAX_SATURATION, ConeDetectionConfig.RED_MAX_VALUE);
            blueLower = new Scalar(ConeDetectionConfig.BLUE_MIN_HUE, ConeDetectionConfig.BLUE_MIN_SATURATION, ConeDetectionConfig.BLUE_MIN_VALUE);
            blueUpper = new Scalar(ConeDetectionConfig.BLUE_MAX_HUE, ConeDetectionConfig.BLUE_MAX_SATURATION, ConeDetectionConfig.BLUE_MAX_VALUE);
        }
        else if (Objects.equals(ConeDetectionConfig.spectrum, "HLS")) {
            redLower = new Scalar(ConeDetectionConfig.RED_MIN_HUE, ConeDetectionConfig.RED_MIN_LIGHTNESS, ConeDetectionConfig.RED_MIN_SATURATION);
            redUpper = new Scalar(ConeDetectionConfig.RED_MAX_HUE, ConeDetectionConfig.RED_MAX_LIGHTNESS, ConeDetectionConfig.RED_MAX_SATURATION);
            blueLower = new Scalar(ConeDetectionConfig.BLUE_MIN_HUE, ConeDetectionConfig.BLUE_MIN_LIGHTNESS, ConeDetectionConfig.BLUE_MIN_SATURATION);
            blueUpper = new Scalar(ConeDetectionConfig.BLUE_MAX_HUE, ConeDetectionConfig.BLUE_MAX_LIGHTNESS, ConeDetectionConfig.BLUE_MAX_SATURATION);
        }

        this.cones = new ArrayList<>();
    }

    @Override
    public Mat processFrame(Mat frame) {

        if (ConeDetectionConfig.updateColors) {
            if (Objects.equals(ConeDetectionConfig.spectrum, "HSV")) {
                redLower = new Scalar(ConeDetectionConfig.RED_MIN_HUE, ConeDetectionConfig.RED_MIN_SATURATION, ConeDetectionConfig.RED_MIN_VALUE);
                redUpper = new Scalar(ConeDetectionConfig.RED_MAX_HUE, ConeDetectionConfig.RED_MAX_SATURATION, ConeDetectionConfig.RED_MAX_VALUE);
                blueLower = new Scalar(ConeDetectionConfig.BLUE_MIN_HUE, ConeDetectionConfig.BLUE_MIN_SATURATION, ConeDetectionConfig.BLUE_MIN_VALUE);
                blueUpper = new Scalar(ConeDetectionConfig.BLUE_MAX_HUE, ConeDetectionConfig.BLUE_MAX_SATURATION, ConeDetectionConfig.BLUE_MAX_VALUE);
            } else if (Objects.equals(ConeDetectionConfig.spectrum, "HLS")) {
                redLower = new Scalar(ConeDetectionConfig.RED_MIN_HUE, ConeDetectionConfig.RED_MIN_LIGHTNESS, ConeDetectionConfig.RED_MIN_SATURATION);
                redUpper = new Scalar(ConeDetectionConfig.RED_MAX_HUE, ConeDetectionConfig.RED_MAX_LIGHTNESS, ConeDetectionConfig.RED_MAX_SATURATION);
                blueLower = new Scalar(ConeDetectionConfig.BLUE_MIN_HUE, ConeDetectionConfig.BLUE_MIN_LIGHTNESS, ConeDetectionConfig.BLUE_MIN_SATURATION);
                blueUpper = new Scalar(ConeDetectionConfig.BLUE_MAX_HUE, ConeDetectionConfig.BLUE_MAX_LIGHTNESS, ConeDetectionConfig.BLUE_MAX_SATURATION);
            }
        }

//        this.undistort(frame);
        Mat mask = new Mat();
        this.filter(frame, mask);
        this.morphology(mask);
        frame.setTo(BLACK, mask);
        ArrayList<Cone> unrankedCones = new ArrayList<>();
        this.findCones(mask, unrankedCones);
        this.rank(unrankedCones);
        this.hud(frame, unrankedCones);
//        Dashboard.packet.put("cones", this.cones.size());
        return frame;
    }

    private void hud(Mat frame, ArrayList<Cone>unrankedCones) {
        for (int i=0;i<this.cones.size();i++) {
            Cone cone = this.cones.get(i);
            if (i==0) { markerOutlined(frame, cone.top, new Point(0,0), GREEN, Imgproc.MARKER_STAR,10,2); }
            if (i==1) { markerOutlined(frame, cone.top, new Point(0,0), ORANGE, Imgproc.MARKER_STAR,10,2); }
            textOutlined(frame, String.valueOf(i), cone.top,new Point(-4,-16), .7, WHITE, 2);
        }
        for (Cone cone: unrankedCones) {
            textOutlined(frame, String.valueOf(new Size(cone.position.dx,cone.position.dy)), cone.top,new Point(0,20),.7, WHITE, 2);
            if (cone.classification == Cone.Classification.CLOSE) {
                markerOutlined(frame, cone.top, new Point(0,0), RED, Imgproc.MARKER_TILTED_CROSS,40,3);
                textOutlined(frame, "CLOSE", cone.top,new Point(0,20), .7, RED, 2);
            } else if (cone.classification == Cone.Classification.FAR) {
                markerOutlined(frame, cone.top, new Point(0,0), RED, Imgproc.MARKER_TILTED_CROSS,40,3);
                textOutlined(frame, "FAR", cone.top,new Point(0,20), .7, RED, 2);
            }
        }
    }
    private void undistort(Mat frame) {
        Mat resizing = new Mat();
        Mat undistorted = new Mat();
        Imgproc.resize(frame, resizing, this.camera.nativeResolution);
        Calib3d.undistort(resizing, undistorted, this.newIntrinsic, this.distortions);
        Imgproc.resize(undistorted, frame, this.camera.resolution);
        resizing.release();
        undistorted.release();
    }

    private void filter(Mat frame, Mat mask) {
        Mat HLS = new Mat();
        if (this.team == Team.RED) {
            if (Objects.equals(ConeDetectionConfig.spectrum, "HSV")) Imgproc.cvtColor(frame, HLS, Imgproc.COLOR_BGR2HSV_FULL);
            else if (Objects.equals(ConeDetectionConfig.spectrum, "HLS")) Imgproc.cvtColor(frame, HLS, Imgproc.COLOR_BGR2HLS_FULL);
            Core.inRange(HLS, redLower, redUpper, mask);
        } else if (this.team == Team.BLUE) {
            if (Objects.equals(ConeDetectionConfig.spectrum, "HSV")) Imgproc.cvtColor(frame, HLS, Imgproc.COLOR_RGB2HSV_FULL);
            else if (Objects.equals(ConeDetectionConfig.spectrum, "HLS")) Imgproc.cvtColor(frame, HLS, Imgproc.COLOR_RGB2HLS_FULL);
            Core.inRange(HLS, blueLower, blueUpper, mask);
        }
        HLS.release();
    }

    private void morphology(Mat mask) {
        Imgproc.morphologyEx(mask,mask,Imgproc.MORPH_OPEN, this.structuringLarge);
        Imgproc.morphologyEx(mask,mask,Imgproc.MORPH_CLOSE, this.structuringMedium);
    }

    private void findCones(Mat mask, ArrayList<Cone> unrankedCones) {

        ArrayList<MatOfPoint> rawContours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, rawContours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();
        mask.release();

        for (MatOfPoint contour: rawContours) {
            Rect rect= Imgproc.boundingRect(contour);
            Point point;
            if (this.conePointMethod == ConePointMethod.MASS) { point = new Point(Imgproc.boundingRect(contour).x + Imgproc.boundingRect(contour).width *.5,Imgproc.boundingRect(contour).y); }
            else { point = getTop(contour); }
            if (Imgproc.contourArea(contour) >= ConeDetectionConfig.coneMinArea && rect.height > rect.width) {
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
                if (rotatedRect.size.height > rotatedRect.size.width) {
                    rotatedRect.size = new Size(rotatedRect.size.height ,rotatedRect.size.width); // <- don't listen to android studio its supposed to be like this
                }
                double angleDistanceCorrection =  ConeDetectionConfig.MAX_DISTANCE_ANGLE_CORRECTION_ADD * Math.abs(this.getAngle(point) / Math.toRadians(30));
                double angleDistanceCorrectionMult =  1 + ConeDetectionConfig.MAX_DISTANCE_ANGLE_CORRECTION_ADD_MULT * Math.abs(this.getAngle(point) / Math.toRadians(30));
                double dist = angleDistanceCorrection +  angleDistanceCorrectionMult * (this.getDistance(rotatedRect.size.width, CONE_WIDTH));
                unrankedCones.add(new Cone(rotatedRect.size, new CameraBasedPosition(dist, this.getAngle(point), this.camera.position), point, rotatedRect));
                contour.release();
                contour2f.release();
            }
        }
    }

    private void textOutlined(Mat frame, String text, Point point, Point offset, double scale, Scalar color, int thickness) {
        Point pos = new Point(point.x+offset.x, point.y+ offset.y);
        Imgproc.putText(frame, text, pos, Imgproc.FONT_HERSHEY_COMPLEX, scale, BLACK,thickness+2);
        Imgproc.putText(frame, text, pos, Imgproc.FONT_HERSHEY_COMPLEX, scale, color,thickness);
    }

    private void markerOutlined(Mat frame,Point point, Point offset, Scalar color,int marker, int size, int thickness) {
        Point pos = new Point(point.x + offset.x, point.y + offset.y);
        Imgproc.drawMarker(frame, pos, BLACK, marker, size, thickness+3);
        Imgproc.drawMarker(frame, pos, color, marker, size, thickness);
    }

    private void rank(ArrayList<Cone> unrankedCones) {
        this.conestackGuess = null;
        if (unrankedCones.size() > 0) {
            this.cones.clear();
            this.cones = unrankedCones.stream()
                    .filter(cone -> cone.classification == Cone.Classification.GOOD && !cone.deadzoned)
                    .collect(Collectors.toList())
                    .stream().sorted(Comparator.comparing(cone -> cone.score))
                    .collect(Collectors.toList());
            Collections.reverse(this.cones);
            this.conestackGuess = Collections.max(unrankedCones, Comparator.comparing(cone -> cone.size.height));
        }
    }

    private static Point getTop(MatOfPoint contour) { return Collections.min(contour.toList(), Comparator.comparing(h -> h.y)); }

    private static Point getCenter(Size size) { return new Point(size.width/2, size.height/2); }

    private double getDistance(double width, double realWidth) {
        double occupiedFOV = this.camera.HFOV * (width / this.camera.resolution.width);
        return ConeDetectionConfig.distanceCorrectionAdd + ConeDetectionConfig.distanceCorrectionMult * ( (realWidth/2)/Math.tan(occupiedFOV/2) + (realWidth/2) );
    }

    private double getAngle(Point point) {
        return this.camera.HFOV * (point.x / this.camera.resolution.width) - this.camera.HFOV /2;
    }

    public List<Cone> getCones() {
        return this.cones;
    }
}
