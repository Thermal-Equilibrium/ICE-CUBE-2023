package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.visionPipelines.PolePipe;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Vision extends Subsystem {
    public static double camWidth;
    public static double camHeight;
    public static final double FOV = 118;


    OpenCvPipeline pipeline = new PolePipe();


    public static OpenCvCamera webcam;

    //public static FtcDashboard dashboard = FtcDashboard.getInstance();
    //public static Telemetry dashboardTelemetry = dashboard.getTelemetry();


    static final Size low = new Size(320,240);
    static final Size medium = new Size(640,480);
    static final Size high = new Size(1280,720);
    static final Size hd = new Size(1920,1080);

    static Size resolution = medium;


    public static void pauseView(){
        webcam.pauseViewport();
    }
    public static void resumeView(){
        webcam.resumeViewport();
    }
    public static int getFrameCount(){ return webcam.getFrameCount(); }
    public static double getFrameRate(){ return webcam.getFps(); }
    public static double getCamWidth(){ return resolution.width; }
    public static double getCamHeight(){ return resolution.height; }
    @Override
    public void initAuto(HardwareMap hwMap) {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming( (int) resolution.width, (int) resolution.height,OpenCvCameraRotation.SIDEWAYS_RIGHT); //OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });

        dashboard.startCameraStream(webcam,30);
        //pauseView();

    }

    @Override
    public void periodic() {
//        Dashboard.packet.put("Vision FPS", webcam.getFps());
//        Dashboard.packet.put("Vision Frame", webcam.getFrameCount());
//        Dashboard.packet.put("Vision Overhead (ms)", webcam.getOverheadTimeMs());
    }

    @Override
    public void shutdown() {
        webcam.closeCameraDevice();
    }
}
