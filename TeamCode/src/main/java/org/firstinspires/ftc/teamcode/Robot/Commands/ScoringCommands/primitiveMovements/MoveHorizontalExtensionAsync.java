package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;

public class MoveHorizontalExtensionAsync extends MoveHorizontalExtension{
	public MoveHorizontalExtensionAsync(HorizontalExtension extension, double desiredPosition) {
		super(extension, desiredPosition);
	}
	@Override
	public boolean completed() {
		return true;
	}
}
