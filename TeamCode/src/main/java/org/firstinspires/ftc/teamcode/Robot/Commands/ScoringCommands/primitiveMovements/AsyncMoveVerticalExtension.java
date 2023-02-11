package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;

public class AsyncMoveVerticalExtension extends MoveVerticalExtension {
	public AsyncMoveVerticalExtension(VerticalExtension extension, Drivetrain drivetrain, double desiredPosition) {
		super(extension, desiredPosition, drivetrain);
	}

	@Override
	public boolean completed() {
		return true;
	}
}
