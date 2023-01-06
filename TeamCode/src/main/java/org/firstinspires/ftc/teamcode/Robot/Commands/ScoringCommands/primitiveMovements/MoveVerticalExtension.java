package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;

public class MoveVerticalExtension extends Command {

	VerticalExtension extension;
	double desiredPosition;

	public MoveVerticalExtension(VerticalExtension extension, double desiredPosition) {
		this.extension = extension;
		this.desiredPosition = desiredPosition;
	}

	@Override
	public void init() {
		extension.updateTargetPosition(desiredPosition);
	}

	@Override
	public void periodic() {
	}

	@Override
	public boolean completed() {
		return extension.isMovementFinished();
	}

	@Override
	public void shutdown() {

	}
}
