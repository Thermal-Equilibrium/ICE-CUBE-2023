package org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

public class EndAction extends Command {

	ScoringMechanism subsystem;
	boolean hasRun = false;

	public EndAction(ScoringMechanism mechanism) {
		this.subsystem = mechanism;
	}

	@Override
	public void init() {
	}

	@Override
	public void periodic() {
		subsystem.shutdown();
	}

	@Override
	public boolean completed() {
		return false;
	}

	@Override
	public void shutdown() {

	}
}
