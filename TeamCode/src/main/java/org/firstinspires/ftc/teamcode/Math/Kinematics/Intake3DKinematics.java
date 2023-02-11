package org.firstinspires.ftc.teamcode.Math.Kinematics;

public class Intake3DKinematics {
	public static double lengthOfArm = 0.0; // inches

	public static double servoToYawRatio = 0.0; // radians per full servo rotation
	public static double yawOffset = Math.PI; // radians
	public static double servoToPitchRatio = 0.0; // radians per full servo rotation
	public static double pitchOffset = 0.0; // radians

	public static double heightOffset = 0.0; // inches
	public static double lengthOffset = 0.0; // inches

	public static double getHeightOfIntake(double pitch) {
		return lengthOfArm * Math.sin(pitch) + heightOffset;
	}

	public static double get2DLengthOfArm(double pitch) {
		return lengthOfArm * Math.cos(pitch);
	}

	public static double getPitchAngleToTarget(double targetZ) {
		return Math.asin(targetZ / lengthOfArm) - heightOffset;
	}

	public static double getYawAngleToTarget(double targetY, double targetZ) {
		return Math.asin(targetY / get2DLengthOfArm(getPitchAngleToTarget(targetZ)));
	}

	public static double get2DLengthOfArmX(double pitch, double yaw) {
		return lengthOfArm * Math.cos(pitch) * Math.cos(Math.abs(yaw - yawOffset));
	}

	public static double get2DLengthOfArmY(double pitch, double yaw) {
		return lengthOfArm * Math.cos(pitch) * Math.sin(Math.abs(yaw - yawOffset));
	}

	public static double getHorizontalSlideExtensionToTarget(double targetX, double targetY, double targetZ) {
		return targetX - (get2DLengthOfArmX(getPitchAngleToTarget(targetZ), getYawAngleToTarget(targetY, targetZ)) - lengthOffset);
	}
}
