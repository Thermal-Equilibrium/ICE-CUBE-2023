package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;

public class SetHorizontalExtensionInches extends MoveHorizontalExtension {

	public SetHorizontalExtensionInches(HorizontalExtension extension, double desiredPosition) {
		super(extension, desiredPosition);
	}

	@Override
	public void init() {
		extension.setTargetPositionInches(desiredPosition);
	}
}
