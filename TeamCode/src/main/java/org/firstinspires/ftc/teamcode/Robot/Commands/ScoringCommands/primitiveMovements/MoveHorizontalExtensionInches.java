package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;

public class MoveHorizontalExtensionInches extends MoveHorizontalExtension {

	public MoveHorizontalExtensionInches(HorizontalExtension extension, double desiredPosition) {
		super(extension, desiredPosition);
	}

	@Override
	public void init() {
		extension.setTargetPositionInches(desiredPosition);
	}
}
