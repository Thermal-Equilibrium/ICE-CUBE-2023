package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

public class GoToSafeHeight extends Command {

	ScoringMechanism mechanism;
	boolean isFinished = false;

	public GoToSafeHeight(ScoringMechanism mechanism) {
		this.mechanism = mechanism;
	}

	@Override
	public void init() {

		if (mechanism.getState().equals(ScoringMechanism.States.CARRY)) {
			mechanism.GO_TO_SAFE_HEIGHT();
		}

	}

	@Override
	public void periodic() {
		if (mechanism.getState().equals(ScoringMechanism.States.CARRY)) {
			mechanism.GO_TO_SAFE_HEIGHT();
		}
		isFinished = true;
	}

	@Override
	public boolean completed() {
		return isFinished;
	}

	@Override
	public void shutdown() {

	}
}
