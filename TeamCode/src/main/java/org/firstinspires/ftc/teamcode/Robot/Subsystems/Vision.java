package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.visionPipelines.Cam;
import org.firstinspires.ftc.teamcode.visionPipelines.LetsSee;
import org.firstinspires.ftc.teamcode.visionPipelines.MonocularPole;
import org.firstinspires.ftc.teamcode.visionPipelines.Optimized;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Vision extends Subsystem {
    @Config
    public static class VisionConfig {
        public static double focusDistance = 1;
    }

    public OpenCvWebcam webcam;
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    static final Size low = new Size(320,240); // bad aspect ratio don't use
    static final Size medium = new Size(800,448);
    static final Size high = new Size(1280,720);
    static final Size hd = new Size(1920,1080);
    static Size resolution = medium;
    public Cam cam;
    public Optimized pipe;
    private Drivetrain drivetrain;
    static ArrayList<MonocularPole> rawPoles;
    public void pauseView(){
        webcam.pauseViewport();
    }
    public void resumeView(){
        webcam.resumeViewport();
    }

    public Vision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    void run() {

    }
    @Override
    public void initAuto(HardwareMap hwMap) {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
//        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY);
        cam = new Cam(0, resolution, new Pose2d(0,4, Math.toRadians(-30)),Math.toRadians(70.428), resolution,1, webcam);// for 78 dfov
        pipe = new Optimized(cam);
        webcam.setPipeline(pipe);
        webcam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                webcam.startStreaming( (int) resolution.width, (int) resolution.height,OpenCvCameraRotation.UPRIGHT);
//                webcam.startStreaming( (int) resolution.width, (int) resolution.height,OpenCvCameraRotation.SIDEWAYS_LEFT);
                Dashboard.packet.put("CAMCONFIG: max gain", webcam.getGainControl().getMaxGain());
                Dashboard.packet.put("CAMCONFIG: Gain Set Success", webcam.getGainControl().setGain(100));
                Dashboard.packet.put("CAMCONFIG: Exposure Mode Success", webcam.getExposureControl().setMode(ExposureControl.Mode.Manual));
                Dashboard.packet.put("CAMCONFIG: Exposure Time Success", webcam.getExposureControl().setExposure(25L, TimeUnit.MILLISECONDS));
                Dashboard.packet.put("CAMCONFIG: Focus Mode Success", webcam.getFocusControl().setMode(FocusControl.Mode.Fixed));
                Dashboard.packet.put("CAMCONFIG: Focus Distance Success", webcam.getFocusControl().setFocusLength(VisionConfig.focusDistance));
            }
            @Override public void onError(int errorCode) { }
        });



        dashboard.startCameraStream(webcam,5);


        rawPoles = new ArrayList<MonocularPole>();
    }

    @Override
    public void periodic() {
        cam.currentFrame= webcam.getFrameCount();
        if (cam.currentFrame> cam.lastFrame) { run(); }
        Dashboard.packet.put("CAMCONFIG: Focus Distance Success", webcam.getFocusControl().setFocusLength(VisionConfig.focusDistance));
        cam.lastFrame= cam.currentFrame;
        Dashboard.packet.put("Cam FPS", webcam.getFps());
        Dashboard.packet.put("Cam Frame", webcam.getFrameCount());

    }

    @Override
    public void shutdown() {
        webcam.closeCameraDevice();
    }


}
