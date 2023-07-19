package org.firstinspires.ftc.teamcode.visionPipelines;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.BackCamera;
import org.firstinspires.ftc.teamcode.Purepursuit.Utils.Team;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.openftc.easyopencv.OpenCvPipeline;

public class Save extends OpenCvPipeline {
	private final Team team;
	private final BackCamera camera;
	private final Point camCenter;
	private int index;
	private final ElapsedTime timer;
	public Save(Team team, BackCamera backCamera) {
		this.team = team;
		this.camera = backCamera;
		this.camCenter = getCenter(this.camera.resolution);
		this.index = 0;
		this.timer = new ElapsedTime(0);
	}

	private static Point getCenter(Size size) {
		return new Point(size.width / 2, size.height / 2);
	}

	@Override
	public Mat processFrame(Mat input) {
		String path = ImgSave.folder + ImgSave.file + index + "." + ImgSave.fileExt;
		boolean hasPath = Imgcodecs.haveImageWriter(path);
		Dashboard.packet.put("SAVE: path", path);
		Dashboard.packet.put("SAVE: hasPath?", hasPath);
		if (hasPath && index < ImgSave.maxSaves && timer.time() >= .5) {
			boolean successfulSave = Imgcodecs.imwrite(path, input);
			Dashboard.packet.put("dir", System.getProperty("user.dir"));
			Dashboard.packet.put("SAVE: successful save?", successfulSave);
			if (successfulSave) {
				index += 1;
				timer.reset();
			}
		}
		return input;
	}

	@Config
	public static class ImgSave {
		public static String folder = "/sdcard/FIRST/";
		public static String file = "img";
		public static String fileExt = "jpg";
		public static int maxSaves = 50;
	}
}
