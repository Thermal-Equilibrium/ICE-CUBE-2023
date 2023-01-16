package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.visionPipelines.Cam;

import org.firstinspires.ftc.teamcode.visionPipelines.ConeDetection;
import org.firstinspires.ftc.teamcode.visionPipelines.SleeveDetection;
import org.firstinspires.ftc.teamcode.Utils.Team;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Vision extends Subsystem {
    @Config
    public static class VisionConfig {
        public static double focusDistance = 1;
    }
    static final Size LOW = new Size(320,240);
    static final Size MEDIUM = new Size(800,448);
    static final Size HIGH = new Size(1280,720);
    static final Size HD = new Size(1920,1080);
    public Cam backCam;
    public Cam frontCam;

    private Team team;

    public Vision(Team team) { this.team = team; }

    @Override
    public void initAuto(HardwareMap hwMap) {
        frontCam = new Cam(new SleeveDetection(), hwMap, "Front Webcam", LOW, HIGH, new Pose2d(0,0, Math.toRadians(0)), Math.toRadians(70.428),OpenCvCameraRotation.SIDEWAYS_LEFT, this.team);
        backCam = new Cam(new ConeDetection(backCam), hwMap,"Back Webcam", MEDIUM, HD, new Pose2d(0,0, Math.toRadians(-30)), Math.toRadians(70.428),OpenCvCameraRotation.UPRIGHT, this.team);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(backCam.webcam, 5);
    }

    @Override
    public void periodic() {
        Dashboard.packet.put("CAMCONFIG: Focus Distance Success", backCam.webcam.getFocusControl().setFocusLength(VisionConfig.focusDistance));
        Dashboard.packet.put("Back Cam FPS", backCam.webcam.getFps());
    }

    @Override
    public void shutdown() {
        frontCam.destroy();
        backCam.destroy();
    }

    public SleeveDetection.ParkingPosition getParkingPosition() {
        if (frontCam.currentFrame < 1 || backCam.destroyed) {
            return SleeveDetection.ParkingPosition.CENTER;
        }
        assert frontCam.pipe instanceof SleeveDetection;
        return ((SleeveDetection) frontCam.pipe).getPosition();
    }
}
