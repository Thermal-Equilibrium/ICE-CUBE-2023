package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;

public class MoveVerticalExtensionTeleop extends Command {

	VerticalExtension extension;
	double desiredPosition;
	double distanceFromPole = 3; //inches
	double initialPosition = 0;

	public MoveVerticalExtensionTeleop(VerticalExtension extension, double desiredPosition) {
		super(extension);
		this.extension = extension;
		this.desiredPosition = desiredPosition;

	}

	@Override
	public void init() {
		extension.updateTargetPosition(desiredPosition);
		this.initialPosition = extension.getSlidePosition();
	}

	@Override
	public void periodic() {
		double distanceToStart = Math.abs(extension.getSlidePosition() - initialPosition);
		double distanceToEnd = Math.abs(extension.getSlidePosition() - desiredPosition);
		if (extension.currentLimitExceeded() && distanceToEnd > distanceToStart) {
			extension.updateTargetPosition(initialPosition);
		}
 	}

	@Override
	public boolean completed() {
		return extension.isMovementFinished();
	}

	@Override
	public void shutdown() {

	}
}
