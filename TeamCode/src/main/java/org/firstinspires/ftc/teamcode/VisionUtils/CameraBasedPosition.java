package org.firstinspires.ftc.teamcode.VisionUtils;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class CameraBasedPosition {
	public double distance;
	public double dy;
	public double dx;
	public double angle;
	public Pose2d cameraPosition;

	public CameraBasedPosition(double distance, double angle, Pose2d cameraPosition) {
		this.cameraPosition = cameraPosition;
		this.distance = distance;
		this.angle = angle + cameraPosition.getHeading();
		if (this.angle < 0) {
			this.angle += Math.toRadians(360);
		}
		if (this.angle > Math.toRadians(360)) {
			this.angle -= Math.toRadians(360);
		}
		this.dx = Math.sin(this.angle) * this.distance;
		this.dy = Math.cos(this.angle) * this.distance;
	}

	public Pose2d toPoseRobotRelative() { // +y is forward, +x is right
		return new Pose2d(this.cameraPosition.getX() + this.dx, this.cameraPosition.getY() - this.dy);
	}

}
