package org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision;




import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.VisionUtils.Resolution;
import org.firstinspires.ftc.teamcode.visionPipelines.SleeveDetection;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class FrontCamera extends Subsystem {
	private final OpenCvCameraRotation cameraRotation = OpenCvCameraRotation.UPRIGHT;
	private final ExposureControl.Mode exposureMode = ExposureControl.Mode.Auto;
	private final long exposureMs = 15;
	private final int gain = 0;
	public Size resolution = Resolution.LOW;
	public double FOV = Math.toRadians(0); // <-(not the actual fov (duh))
	public Pose2d position = new Pose2d(0, 0, Math.toRadians(0));
	private final OpenCvPipeline pipeline;
	private OpenCvWebcam cam;
	private boolean open = false;

	public FrontCamera() {
		pipeline = new SleeveDetection();
	}

	@Override
	public void initAuto(HardwareMap hwMap) {
//		cam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Front Webcam"));

		cam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "usbCamera"));

		cam.setPipeline(pipeline);
		cam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				open = true;
				cam.startStreaming((int) resolution.width, (int) resolution.height, cameraRotation);
//                cam.getExposureControl().setMode(exposureMode);
//                if (exposureMode == ExposureControl.Mode.Manual) {
//                    cam.getExposureControl().setExposure(exposureMs, TimeUnit.MILLISECONDS);
//                }
//                cam.getGainControl().setGain(gain);
			}

			@Override
			public void onError(int errorCode) {
			}
		});
		FtcDashboard dashboard = FtcDashboard.getInstance();
		dashboard.startCameraStream(cam, 20);

	}

	@Override
	public void periodic() {
	}

	@Override
	public void shutdown() {
		this.close();
	}

	public void close() {
		open = false;
		cam.closeCameraDevice();
	}

	public SleeveDetection.ParkingPosition getParkingPosition() {
		if (cam.getFrameCount() < 1 || !open) {
			return SleeveDetection.ParkingPosition.LEFT;
		}
		assert pipeline instanceof SleeveDetection;
		return ((SleeveDetection) pipeline).getPosition();
	}
}
