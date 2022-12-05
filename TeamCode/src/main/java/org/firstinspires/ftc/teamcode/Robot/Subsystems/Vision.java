package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.visionPipelines.Cam;
import org.firstinspires.ftc.teamcode.visionPipelines.OdometryPipe;
import org.firstinspires.ftc.teamcode.visionPipelines.MonocularPole;
import org.firstinspires.ftc.teamcode.visionPipelines.StereoPole;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class Vision extends Subsystem {

    public static OpenCvCamera webcam1;
    public static OpenCvCamera webcam2;

    public static FtcDashboard dashboard = FtcDashboard.getInstance();

    static final Size low = new Size(320,240); // bad aspect ratio don't use
    static final Size medium = new Size(960,540);
    static final Size high = new Size(1280,720);
    static final Size hd = new Size(1920,1080);

    static Size resolution = medium;

    public Cam cam1 = new Cam(0, resolution, new Size(0,4),Math.toRadians(70.428), new Size(1920,1080),1);// for 78 dfov
    public Cam cam2 = new Cam(1, resolution, new Size(4,4),Math.toRadians(70.428),new Size(1280,720),1.3);

    OpenCvPipeline pipeline1 = new OdometryPipe(cam1);
    OpenCvPipeline pipeline2 = new OdometryPipe(cam2);

    double poleIsPoleThresh = .075;
    ArrayList<StereoPole> stereoPoles = new ArrayList<StereoPole>();

    static ArrayList<MonocularPole> rawPoles;
    Drivetrain drivetrain;

    public static void pauseView(){
        webcam1.pauseViewport();
        webcam2.pauseViewport();
    }
    public static void resumeView(){
        webcam1.resumeViewport();
        webcam1.resumeViewport();
    }

    public Vision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    void processStereo() {
        // poles are poles are poles?
        stereoPoles.clear();
        for (int i1 = 0; i1 < cam1.detectedPoles.size(); i1++) {
            for (int i2 = 0; i2 < cam1.detectedPoles.size(); i2++) {
                double yError = Math.abs( ( cam1.detectedPoles.get(i1).dEstimate.height + cam1.position.height ) - ( cam2.detectedPoles.get(i2).dEstimate.height + cam2.position.height ) );
                double xError = Math.abs( ( cam1.detectedPoles.get(i1).dEstimate.width + cam1.position.width ) - ( cam2.detectedPoles.get(i2).dEstimate.width + cam2.position.width ) );
                if (yError<=poleIsPoleThresh && xError <= poleIsPoleThresh){
                    stereoPoles.add(new StereoPole(cam1.detectedPoles.get(i1),cam2.detectedPoles.get(i2)));
                }
            }
        }
        // triangulation

        //trilateration
    }
    void run() {
        processStereo();
    }
    @Override
    public void initAuto(HardwareMap hwMap) {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 2"));

        webcam1.setPipeline(pipeline1);
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() { webcam1.startStreaming( (int) resolution.width, (int) resolution.height,OpenCvCameraRotation.UPRIGHT); }
            @Override public void onError(int errorCode) { }
        });

        webcam2.setPipeline(pipeline2);
        webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() { webcam2.startStreaming( (int) resolution.width, (int) resolution.height,OpenCvCameraRotation.UPRIGHT); }
            @Override public void onError(int errorCode) { }
        });

        dashboard.startCameraStream(webcam1,5);

        rawPoles = new ArrayList<MonocularPole>();
    }

    @Override
    public void periodic() {
        cam1.currentFrame=webcam1.getFrameCount();
        cam2.currentFrame=webcam2.getFrameCount();
        if (cam1.currentFrame>cam1.lastFrame | cam2.currentFrame>cam2.lastFrame) { run(); }
        cam1.lastFrame=cam1.currentFrame;
        cam2.lastFrame=cam2.currentFrame;

        Dashboard.packet.put("Cam1 FPS", webcam1.getFps());
        Dashboard.packet.put("Cam1 Frame", webcam1.getFrameCount());
        Dashboard.packet.put("Cam2 FPS", webcam2.getFps());
        Dashboard.packet.put("Cam2 Frame", webcam2.getFrameCount());
    }

    @Override
    public void shutdown() {
        webcam1.closeCameraDevice();
        webcam2.closeCameraDevice();
    }
}
