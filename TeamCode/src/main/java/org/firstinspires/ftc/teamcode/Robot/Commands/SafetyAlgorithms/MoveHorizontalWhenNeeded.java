package org.firstinspires.ftc.teamcode.Robot.Commands.SafetyAlgorithms;

import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveHorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;

import java.util.function.BooleanSupplier;

public class MoveHorizontalWhenNeeded extends MoveHorizontalExtension {

	BooleanSupplier externalStop;

	public MoveHorizontalWhenNeeded(HorizontalExtension extension, double desiredPosition, BooleanSupplier externalStop) {
		super(extension, desiredPosition);
		this.externalStop = externalStop;
	}

	@Override
	public void init() {
		if (!externalStop.getAsBoolean()) {
			extension.setTargetPosition(desiredPosition);
		}
	}

	@Override
	public boolean completed() {
		return extension.isMovementFinished() || extension.currentExceedsCutoff() || externalStop.getAsBoolean();
	}
}
