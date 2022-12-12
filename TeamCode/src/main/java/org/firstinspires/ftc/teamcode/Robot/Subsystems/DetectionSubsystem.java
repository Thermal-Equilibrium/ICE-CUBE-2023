package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.dashboard;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.visionPipelines.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class DetectionSubsystem extends Subsystem {
	private SleeveDetection sleeveDetection;
	private OpenCvCamera webcam1;
	private boolean hasInitialized = false;
	private boolean isDestroyed = false;
	public FtcDashboard dash;

	public DetectionSubsystem(FtcDashboard dash) {
		this.dash = dash;
	}

	@Override
	public void initAuto(HardwareMap hwMap) {
		int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
		webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
		sleeveDetection = new SleeveDetection();
		webcam1.setPipeline(sleeveDetection);
		webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override public void onOpened() { webcam1.startStreaming( 320,240, OpenCvCameraRotation.SIDEWAYS_LEFT); }
			@Override public void onError(int errorCode) { }
		});
		hasInitialized = true;
		dash.startCameraStream(webcam1,5);

	}

	@Override
	public void periodic() {

	}

	@Override
	public void shutdown() {

	}

	public SleeveDetection.ParkingPosition getPosition() {
		if (!hasInitialized || isDestroyed) {
			return SleeveDetection.ParkingPosition.CENTER;
		}
		return sleeveDetection.getPosition();
	}

	public void destroy() {
		isDestroyed = true;
		webcam1.closeCameraDevice();
	}
}
