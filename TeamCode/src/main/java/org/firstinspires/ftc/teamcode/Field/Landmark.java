package org.firstinspires.ftc.teamcode.Field;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public abstract class Landmark {
    Pose2d pose;
    public Landmark(Pose2d pose) {
        pose = pose;
    }
}
