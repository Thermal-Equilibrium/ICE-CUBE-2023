package org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.teamcode.VisionUtils.Cone;
import org.firstinspires.ftc.teamcode.VisionUtils.Resolution;
import org.firstinspires.ftc.teamcode.visionPipelines.ConeDetectionFast;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

public class BackCamera extends Subsystem {
    public Size resolution = Resolution.LOW;
    public double FOV = Math.toRadians(70.428);
    public Pose2d position = new Pose2d(0,0, Math.toRadians(180)); // TODO: make this update with slide extension
    private OpenCvPipeline pipeline;
    private OpenCvWebcam cam;
    private final OpenCvCameraRotation cameraRotation = OpenCvCameraRotation.UPRIGHT;
    private final ExposureControl.Mode exposureMode = ExposureControl.Mode.Manual;
    private final long exposureMs = 15;
    private final int gain = 0;
    private final FocusControl.Mode focusMode = FocusControl.Mode.Fixed;
    private final double focusLength = 69; //idk what units this is in
    private Cone tempCone;

    public BackCamera(Team team) {
        pipeline = new ConeDetectionFast(team,this);
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        cam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Back Webcam"), hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName()));
        cam.setViewportRenderer(OpenCvWebcam.ViewportRenderer.GPU_ACCELERATED);
        cam.setPipeline(pipeline);
        cam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                cam.startStreaming( (int) resolution.width, (int) resolution.height, cameraRotation);
                cam.getExposureControl().setMode(exposureMode);
                cam.getExposureControl().setExposure(exposureMs, TimeUnit.MILLISECONDS);
                cam.getGainControl().setGain(gain);
                cam.getFocusControl().setMode(focusMode);
                if (focusMode == FocusControl.Mode.Fixed) {
                    //cam.getFocusControl().setFocusLength(focusLength);
                }

            }
            @Override public void onError(int errorCode) { }});
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(cam, 20);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void shutdown() {
        cam.closeCameraDevice();
    }

    @Nullable
    public Cone getCone(boolean allowFar,boolean allowClose) {
        assert pipeline instanceof ConeDetectionFast;
        tempCone = ((ConeDetectionFast) pipeline).perfect;
        if (tempCone != null) return tempCone;

        if (allowFar) {
            tempCone = ((ConeDetectionFast) pipeline).far;
            if (tempCone != null) return tempCone;
        }
        if (allowClose) {
            tempCone = ((ConeDetectionFast) pipeline).close;
            if (tempCone != null) return tempCone;
        }
        return null;
    }

    @Nullable
    public Cone getConeStack() {
        assert pipeline instanceof ConeDetectionFast;
        return ((ConeDetectionFast) pipeline).conestackGuess;
    }


}
