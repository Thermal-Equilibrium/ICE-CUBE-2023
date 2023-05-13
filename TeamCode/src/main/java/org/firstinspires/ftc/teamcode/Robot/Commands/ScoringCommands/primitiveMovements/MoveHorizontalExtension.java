package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;

public class MoveHorizontalExtension extends Command {
	protected HorizontalExtension extension;
	protected double desiredPosition;

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
		return extension.isMovementFinished() || extension.currentExceedsCutoff();
	}

	@Override
	public void shutdown() {

	}

}
