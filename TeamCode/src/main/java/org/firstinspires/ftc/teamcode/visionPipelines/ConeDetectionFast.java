package org.firstinspires.ftc.teamcode.visionPipelines;


import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.BackCamera;
import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.teamcode.VisionUtils.CameraBasedPosition;
import org.firstinspires.ftc.teamcode.VisionUtils.Cone;
import org.firstinspires.ftc.teamcode.VisionUtils.ConePointMethod;
import org.firstinspires.ftc.teamcode.VisionUtils.VisionMode;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

public class ConeDetectionFast extends OpenCvPipeline {
	private static final double CONE_WIDTH = 4;
	private static final Scalar BLANK = new Scalar(0, 0, 0);
	private static final Scalar RED = new Scalar(255, 0, 0);
	private static final Scalar ORANGE = new Scalar(255, 165, 0);
	private static final Scalar YELLOW = new Scalar(255, 255, 0);
	private static final Scalar GREEN = new Scalar(0, 255, 0);
	private static final Scalar BLUE = new Scalar(0, 0, 255);
	private static final Scalar PURPLE = new Scalar(255, 0, 255);
	private static final Scalar BLACK = new Scalar(1, 1, 1);
	private static final Scalar WHITE = new Scalar(255, 255, 255);
	private final Mat structuringSmall = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(3, 3));
	private final Mat structuringMedium = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(5, 5));
	private final Mat structuringLarge = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(7, 7));
	public ConePointMethod conePointMethod;
	public volatile Cone conestackGuess = null;
	Scalar redLower;
	Scalar redUpper;
	Scalar blueLower;
	Scalar blueUpper;
	private final BackCamera camera;
	private final Point camCenter;
	private Team team;
	private VisionMode visionMode;

	private volatile List<Cone> cones;
	public ConeDetectionFast(Team team, VisionMode visionMode, BackCamera backCamera) {
		this.team = team;
		this.visionMode = visionMode;
		this.camera = backCamera;
		this.camCenter = getCenter(this.camera.resolution);
		this.conePointMethod = ConePointMethod.MASS;
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
		this.cones = new ArrayList<>();
	}

	private static Point getTop(MatOfPoint contour) {
		return Collections.min(contour.toList(), Comparator.comparing(h -> h.y));
	}

	private static Point getPoint(MatOfPoint contour, Mat mask) {
		Rect rect = Imgproc.boundingRect(contour);
		rect.height = (int) (rect.height*.1);
		Mat roi = mask.submat(rect);
		ArrayList<MatOfPoint> contours2 = new ArrayList<>();
		Imgproc.findContours(roi,contours2,new Mat(),Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
		Rect rect2 = Imgproc.boundingRect(contours2.get(0));
//		Point point2 = new Point(rect2.x,rect2));
		Moments m = Imgproc.moments(contours2.get(0));
		Point center = new Point(m.get_m10() / m.get_m00(), m.get_m01() / m.get_m00());
		roi.release();
		return new Point(rect.br().x - center.x,rect.br().y - center.y);
	}

	private static Point getCenter(Size size) {
		return new Point(size.width / 2, size.height / 2);
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

        //this.undistort(frame);
		Mat mask = new Mat();
		Imgproc.rectangle(frame,new Point(0,frame.rows()), new Point(frame.cols(),frame.rows()*(1-ConeDetectionConfig.cutOff)),GREEN,-1);
		this.filter(frame, mask);
		this.morphology(mask);

		frame.setTo(PURPLE, mask);
		ArrayList<Cone> unrankedCones = new ArrayList<>();
		this.findCones(mask, unrankedCones);
		this.rank(unrankedCones);
		this.hud(frame, unrankedCones);
		return frame;
	}

	private void hud(Mat frame, ArrayList<Cone> unrankedCones) {
		for (int i = 0; i < this.cones.size(); i++) {
			Cone cone = this.cones.get(i);
			if (i == 0) {
				markerOutlined(frame, cone.point, new Point(0, 0), GREEN, Imgproc.MARKER_STAR, 10, 2);
			}
			if (i == 1) {
				markerOutlined(frame, cone.point, new Point(0, 0), ORANGE, Imgproc.MARKER_STAR, 10, 2);
			}

//			textOutlined(frame, round2Decimal(cone.position.dx), cone.point, new Point(-20, 10), .8, WHITE, 2);
//			textOutlined(frame, round2Decimal(cone.position.dy), cone.point, new Point(-20, 40), .8, WHITE, 2);
			textOutlined(frame, String.valueOf(i), cone.point, new Point(-4, -16), .7, WHITE, 2);
			textOutlined(frame, round2Decimal(cone.position.dx) + ", " + round2Decimal(cone.position.dy), cone.point, new Point(0, 20), .7, WHITE, 2);
			textOutlined(frame, round2Decimal(cone.position.distance), cone.point, new Point(-4, 40), .7, WHITE, 2);
			textOutlined(frame, round2Decimal(cone.position.angle), cone.point, new Point(-4, 60), .7, WHITE, 2);
		}
		for (Cone cone : unrankedCones) {

			if (cone.classification == Cone.Classification.CLOSE) {
				markerOutlined(frame, cone.point, new Point(0, 0), RED, Imgproc.MARKER_TILTED_CROSS, 40, 3);
				textOutlined(frame, "CLOSE", cone.point, new Point(0, 20), .7, RED, 2);
			} else if (cone.classification == Cone.Classification.FAR) {
				markerOutlined(frame, cone.point, new Point(0, 0), RED, Imgproc.MARKER_TILTED_CROSS, 40, 3);
				textOutlined(frame, "FAR", cone.point, new Point(0, 20), .7, RED, 2);
			}
		}
	}

	private void filter(Mat frame, Mat mask) {
		Mat HLS = new Mat();
		if (this.team == Team.RED) {
			if (Objects.equals(ConeDetectionConfig.spectrum, "HSV"))
				Imgproc.cvtColor(frame, HLS, Imgproc.COLOR_BGR2HSV_FULL);
			else if (Objects.equals(ConeDetectionConfig.spectrum, "HLS"))
				Imgproc.cvtColor(frame, HLS, Imgproc.COLOR_BGR2HLS_FULL);
			Core.inRange(HLS, redLower, redUpper, mask);
		} else if (this.team == Team.BLUE) {
			if (Objects.equals(ConeDetectionConfig.spectrum, "HSV"))
				Imgproc.cvtColor(frame, HLS, Imgproc.COLOR_RGB2HSV_FULL);
			else if (Objects.equals(ConeDetectionConfig.spectrum, "HLS"))
				Imgproc.cvtColor(frame, HLS, Imgproc.COLOR_RGB2HLS_FULL);
			Core.inRange(HLS, blueLower, blueUpper, mask);
		} else {
			if (Objects.equals(ConeDetectionConfig.spectrum, "HSV"))
				Imgproc.cvtColor(frame, HLS, Imgproc.COLOR_BGR2HSV_FULL);
			else if (Objects.equals(ConeDetectionConfig.spectrum, "HLS"))
				Imgproc.cvtColor(frame, HLS, Imgproc.COLOR_BGR2HLS_FULL);
			Core.inRange(HLS, redLower, redUpper, mask);
		}
		HLS.release();
	}

	private void morphology(Mat mask) {
		Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, this.structuringMedium);
		Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, this.structuringMedium);
	}

	private void findCones(Mat mask, ArrayList<Cone> unrankedCones) {

		ArrayList<MatOfPoint> rawContours = new ArrayList<>();
		Mat hierarchy = new Mat();
		Imgproc.findContours(mask, rawContours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
		hierarchy.release();

		for (MatOfPoint contour : rawContours) {
			Rect rect = Imgproc.boundingRect(contour);
			if (Imgproc.contourArea(contour) >= ConeDetectionConfig.coneMinArea && rect.height > rect.width) {
				Point point;

//				point = getPoint(contour,mask);

				if (this.conePointMethod == ConePointMethod.MASS) {
					point = new Point(Imgproc.boundingRect(contour).x + Imgproc.boundingRect(contour).width * .5, Imgproc.boundingRect(contour).y);
				} else {
					point = getTop(contour);
				}

				double dist = this.getDistance(rect.width, CONE_WIDTH);
				double angle = this.getAngle(point);

				if (angle >= Math.PI) angle -= Math.PI*2; //unfix angle
				double[] z = {1, dist, angle, Math.pow(dist, 2), dist * angle, Math.pow(angle, 2), Math.pow(dist, 3), Math.pow(dist, 2) * angle, dist * Math.pow(angle, 2), Math.pow(angle, 3)};
				double yhat = 0;
				double[] params = {6.31364828e+00, -1.12473013e+00, -4.80012098e+00,  4.20113244e-02,
						3.34183578e-01,  2.92658270e+00, -5.71336671e-04, -3.47121640e-03,
						8.20912273e-01, -1.70789303e+00};
				for (int i = 0; i < params.length; i++) {
					yhat += params[i] * z[i];
				}
				dist += yhat;
				if (angle < 0) angle += Math.PI*2; //refix angle
				dist *= ConeDetectionConfig.distMult;
				dist += ConeDetectionConfig.distAdd;;
				unrankedCones.add(new Cone(rect.size(), new CameraBasedPosition(dist, angle, this.camera.position), point));
				contour.release();
			}
		}
		mask.release();
	}

	private void textOutlined(Mat frame, String text, Point point, Point offset, double scale, Scalar color, int thickness) {
		Point pos = new Point(point.x + offset.x, point.y + offset.y);
		Imgproc.putText(frame, text, pos, Imgproc.FONT_HERSHEY_COMPLEX, scale, BLACK, thickness + 2);
		Imgproc.putText(frame, text, pos, Imgproc.FONT_HERSHEY_COMPLEX, scale, color, thickness);
	}

	private void markerOutlined(Mat frame, Point point, Point offset, Scalar color, int marker, int size, int thickness) {
		Point pos = new Point(point.x + offset.x, point.y + offset.y);
		Imgproc.drawMarker(frame, pos, BLACK, marker, size, thickness + 3);
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

	private double getDistance(double width, double realWidth) {
		double occupiedFOV = this.camera.HFOV * (width / this.camera.resolution.width);
		return ((realWidth / 2) / Math.tan(occupiedFOV / 2) + (realWidth / 2));
	}

	private double getAngle(Point point) {
		return this.camera.HFOV * (point.x / this.camera.resolution.width) - this.camera.HFOV / 2;
	}

	public List<Cone> getCones() {
		return this.cones;
	}

	@SuppressLint("DefaultLocale")
	private String round2Decimal(double number){
		return String.format("%.2f",number);
	}

	@Config
	public static class ConeDetectionConfig {
		public static double distMult = 1;
		public static double distAdd = 0;
		public static boolean updateColors = true;
		public static int coneMinArea = 600;
		public static int RED_MIN_HUE = 161;
		public static int RED_MIN_SATURATION = 80;
		public static int RED_MIN_VALUE = 0;
		public static int RED_MIN_LIGHTNESS = 0;
		public static int RED_MAX_HUE = 190;
		public static int RED_MAX_SATURATION = 255;
		public static int RED_MAX_VALUE = 255;
		public static int RED_MAX_LIGHTNESS = 255;
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
		public static double cutOff = .55;
	}
	public void setVisionMode(VisionMode visionMode) {
		this.visionMode = visionMode;
	}
}
