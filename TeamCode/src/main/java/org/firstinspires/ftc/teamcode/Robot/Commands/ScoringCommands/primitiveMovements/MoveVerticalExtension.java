package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;

public class MoveVerticalExtension extends Command {

	VerticalExtension extension;
	double desiredPosition;
	Drivetrain drivetrain;
	double distanceFromPole = 3; //inches

	public MoveVerticalExtension(VerticalExtension extension, double desiredPosition, Drivetrain drivetrain) {
		super(drivetrain);
		this.extension = extension;
		this.desiredPosition = desiredPosition;

		this.drivetrain = drivetrain;
	}

	@Override
	public void init() {
		extension.updateTargetPosition(desiredPosition);
//		if (desiredPosition == VerticalExtension.IN_POSITION) {
//			Pose2d pose =  drivetrain.getPose();
//			// detect the nearest pole using the current position and the angle
//			// the field is -70 to 70 in both axis
//			// the poles are 23.3 inches apart
//
//			// get the nearest center of pole to the robot
//			double x = Math.round(pose.getX() / 23.3) * 23.3;
//			double y = Math.round(pose.getY() / 23.3) * 23.3;
//
//			// get the angle of the robot
//			double robotAngle = pose.getHeading();
//
//			// offset the robot position by the distance from the pole
//			double xOff = x + distanceFromPole * Math.cos(robotAngle);
//			double yOff = y + distanceFromPole * Math.sin(robotAngle);
//
//			// update robot pose
//			drivetrain.setPose(new Pose2d(xOff, yOff, robotAngle));
//		}
	}

	@Override
	public void periodic() {
	}

	@Override
	public boolean completed() {
		return extension.isMovementFinished() || extension.currentLimitExceeded();
	}

	@Override
	public void shutdown() {

	}
}
