package org.firstinspires.ftc.teamcode.visionPipelines;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Pole {
    public Pose2d pose;
    public double verticalAngle;
    public Pole(Pose2d pose, double verticalAngle) {
        this.pose=pose;
        this.verticalAngle=verticalAngle;

    }
}
