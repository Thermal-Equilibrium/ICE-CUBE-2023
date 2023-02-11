package org.firstinspires.ftc.teamcode.Math.Kinematics;

// X points forward, Y points left, and Z points up
// All variables and arguments SHOULD BE IN INCHES AND RADIANS
public class IntakeKinematics {
	// Webcam X offset from the arm pivot point
	private static final double webcamXOffset = 0.3;

	// X (just on the X axis, not slanted) length of arm WHEN IN CONE PICK UP position
	private static final double armXLength = 15;

	// Given X position of a target relative to the intake webcam's reference frame,
	// return the X position of the target relative to the
	// robot reference frame (the arm pivot point when the horizontal slides are fully retracted.
	public static double getXRobotRelativePosition(double targetX, double horizontalSlideExtension) {
		return horizontalSlideExtension + webcamXOffset + targetX;
	}

	// Given targetY offset,
	// returns the angle that turret should be at to pick the target up
	public static double getTurretAngleToTarget(double targetY) {
		return Math.asin(targetY / armXLength);
	}

	// Given the targetX and targetY offsets, and the current horizontal slide extension,
	// returns the position the horizontal slides should be extended to in order to pick the target up
	public static double getHorizontalSlideExtensionToTarget(double targetX, double targetY, double currentHorizontalSlideExtension) {
		// Translate targetX into the robot relative reference frame
		targetX = getXRobotRelativePosition(targetX, currentHorizontalSlideExtension);
		double turretAngle = getTurretAngleToTarget(targetY);

		return targetX - armXLength * Math.cos(turretAngle);
	}
}
