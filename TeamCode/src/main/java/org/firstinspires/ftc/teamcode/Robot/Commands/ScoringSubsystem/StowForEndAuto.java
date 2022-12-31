package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

public class StowForEndAuto extends Command {
	ScoringMechanism scoringMechanism;
	boolean hasRun = false;

	public StowForEndAuto(ScoringMechanism scoringMechanism) {
		this.scoringMechanism = scoringMechanism;
	}

	@Override
	public void init() {
		scoringMechanism.endAuto();
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
