package org.firstinspires.ftc.teamcode.visionPipelines;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

public class Cam {
    public Size nativeRes;
    String name;
    public Pose2d position;

    Mat camMat;
    Mat newCamMat;
    Mat dists;
    public Size res;
    public double FOV;

    public OpenCvWebcam webcam;
    public OpenCvPipeline pipe;
    public double currentFrame;
    public double lastFrame;
    public boolean destroyed = false;

    public Cam(OpenCvPipeline pipe, HardwareMap hwMap, String name, Size res, Size nativeRes, Pose2d position, double FOV, OpenCvCameraRotation rotation) {
        this.name = name;
        this.res = res;
        this.position=position;
        this.FOV=FOV;

        this.nativeRes = nativeRes;

        this.pipe = pipe;

        this.camMat = this.loadCamMatrix();
        this.newCamMat = this.loadNewCamMatrix();
        this.dists = this.loadDists();

        this.currentFrame=0;
        this.lastFrame=0;

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, this.name), cameraMonitorViewId);
        webcam.setViewportRenderer(OpenCvWebcam.ViewportRenderer.GPU_ACCELERATED);

        webcam.setPipeline(this.pipe);

        webcam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                webcam.startStreaming( (int) res.width, (int) res.height, rotation);
                Dashboard.packet.put("CAMCONFIG: max gain", webcam.getGainControl().getMaxGain());
                Dashboard.packet.put("CAMCONFIG: Gain Set Success", webcam.getGainControl().setGain(100));
                Dashboard.packet.put("CAMCONFIG: Exposure Mode Success", webcam.getExposureControl().setMode(ExposureControl.Mode.Manual));
                Dashboard.packet.put("CAMCONFIG: Exposure Time Success", webcam.getExposureControl().setExposure(25L, TimeUnit.MILLISECONDS));
                Dashboard.packet.put("CAMCONFIG: Focus Mode Success", webcam.getFocusControl().setMode(FocusControl.Mode.Fixed));
                Dashboard.packet.put("CAMCONFIG: Focus Distance Success", webcam.getFocusControl().setFocusLength(Vision.VisionConfig.focusDistance));
            }
            @Override public void onError(int errorCode) { }
        });
    }

    private Mat loadCamMatrix() {
        Mat camMat = new Mat(3, 3, CvType.CV_64F, new Scalar(0));
        camMat.put(0, 0, 1426.5954310779582);
        camMat.put(0, 1, 0.0);
        camMat.put(0, 2, 976.06694339774);

        camMat.put(1, 0, 0.0);
        camMat.put(1, 1, 1427.6090838901566);
        camMat.put(1, 2, 523.6314205483386);

        camMat.put(2, 0, 0.0);
        camMat.put(2, 1, 0.0);
        camMat.put(2, 2, 1.0);

        return camMat;
    }

    private Mat loadNewCamMatrix() {
        Mat newCamMat = new Mat(3, 3, CvType.CV_64F, new Scalar(0));

        newCamMat.put(0, 0, 1417.0025634765625);
        newCamMat.put(0, 1, 0.0);
        newCamMat.put(0, 2, 974.1928097073833);

        newCamMat.put(1, 0, 0.0);
        newCamMat.put(1, 1, 1417.5028076171875);
        newCamMat.put(1, 2, 524.6199316240309);

        newCamMat.put(2, 0, 0.0);
        newCamMat.put(2, 1, 0.0);
        newCamMat.put(2, 2, 1.0);

        return newCamMat;
    }

    private Mat loadDists() {
        Mat dists = new Mat(1, 5, CvType.CV_64F, new Scalar(0));
        dists.put(0, 0, 0.05140565587875675);
        dists.put(0, 1, -0.22393758599529429);
        dists.put(0, 2, 0.0011717733816520855);
        dists.put(0, 3, -0.0006126942692220965);
        dists.put(0, 4, 0.20549267503932084);

        return dists;
    }
    public void destroy() {
        if (!this.destroyed) this.webcam.closeCameraDevice();
        this.destroyed = true;
    }



}
