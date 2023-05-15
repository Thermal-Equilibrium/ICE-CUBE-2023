package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;

public class SetSlideWeaken extends Command {

	HorizontalExtension extension;
	boolean weaken;
	boolean complete = false;

	public SetSlideWeaken(HorizontalExtension extension, boolean weaken) {
		this.extension = extension;
		this.weaken = weaken;
	}


	@Override
	public void init() {
		this.extension.weakened = weaken;
		complete = true;
	}

	@Override
	public void periodic() {

	}

	@Override
	public boolean completed() {
		return complete;
	}

	@Override
	public void shutdown() {

	}
}
