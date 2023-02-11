package org.firstinspires.ftc.teamcode.Math.TheArcaneConceptThatIsTurningInPlace;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Heading {
	double angleRR;
	double angleFR;

	/**
	 * Angle that can easily be converted from robot relative to field relative or vice versa
	 *
	 * @param pose          the robot's pose
	 * @param angle         its a fucking angle
	 * @param robotRelative is the angle relative to the robot
	 */
	public Heading(Pose2d pose, double angle, boolean robotRelative) {
		if (robotRelative) {
			this.angleRR = angle;
			this.angleFR = angle + pose.getHeading();
		} else {
			this.angleRR = angle - pose.getHeading();
			this.angleFR = angle;
		}
	}

	public double asRR() {
		return this.angleRR;
	}

	public double asFR() {
		return this.angleFR;
	}
}
