package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

public class openClaw extends Command {

	boolean hasRun = false;
	ScoringMechanism scoringMechanism;
	public openClaw(ScoringMechanism subsystem) {
		super(subsystem);
		this.scoringMechanism = subsystem;
	}

	@Override
	public void init() {
		scoringMechanism.setInPossession(false);
		hasRun = true;
	}

	@Override
	public void periodic() {

	}

	@Override
	public boolean completed() {
		return hasRun;
	}

	@Override
	public void shutdown() {

	}
}
