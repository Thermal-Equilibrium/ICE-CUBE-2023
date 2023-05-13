package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;

public class MoveVerticalExtensionAsync extends MoveVerticalExtension{
	public MoveVerticalExtensionAsync(VerticalExtension extension, double desiredPosition, Drivetrain drivetrain) {
		super(extension, desiredPosition, drivetrain);
	}
	@Override
	public boolean completed() {
		return true;
	}
}
