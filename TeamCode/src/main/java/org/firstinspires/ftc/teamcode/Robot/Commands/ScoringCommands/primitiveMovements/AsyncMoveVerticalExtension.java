package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;

public class AsyncMoveVerticalExtension extends MoveVerticalExtension{
	public AsyncMoveVerticalExtension(VerticalExtension extension, double desiredPosition) {
		super(extension, desiredPosition);
	}
	@Override
	public boolean completed() {
		return true;
	}
}
