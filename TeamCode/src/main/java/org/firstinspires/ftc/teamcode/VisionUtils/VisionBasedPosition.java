package org.firstinspires.ftc.teamcode.VisionUtils;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class VisionBasedPosition {
    public double distance;
    public double angle;
    public Pose2d cameraPosition;

    public VisionBasedPosition(double distance, double angle, Pose2d cameraPosition) {
        this.distance = distance;
        this.angle = angle;
        this.cameraPosition = cameraPosition;
    }

    public Pose2d toPoseRobotRelative() {
        double dxEstimate = Math.sin(this.angle) * this.distance;
        double dyEstimate = Math.cos(this.angle) * this.distance;
        return new Pose2d(this.cameraPosition.getX() + dxEstimate, this.cameraPosition.getY() - dyEstimate);
    }

}
