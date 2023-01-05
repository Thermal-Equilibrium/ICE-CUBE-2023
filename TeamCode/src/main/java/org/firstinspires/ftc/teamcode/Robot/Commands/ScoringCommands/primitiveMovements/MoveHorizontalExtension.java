package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;

public class MoveHorizontalExtension extends Command {
	HorizontalExtension extension;
	double desiredPosition;

	public MoveHorizontalExtension(HorizontalExtension extension, double desiredPosition) {
		this.extension = extension;
		this.desiredPosition = desiredPosition;
	}

	@Override
	public void init() {
		extension.setTargetPosition(desiredPosition);
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
