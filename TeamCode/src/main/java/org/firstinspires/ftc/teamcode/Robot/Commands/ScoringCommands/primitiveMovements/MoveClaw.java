package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.ArmSystem;

public class MoveClaw extends Command {

	double delayS = 0.20;

	ArmSystem armSystem;
	ArmSystem.ClawStates clawState;

	ElapsedTime timer = new ElapsedTime();

	public MoveClaw(ArmSystem armSystem, ArmSystem.ClawStates clawState) {
		this.armSystem = armSystem;
		this.clawState = clawState;
	}

	@Override
	public void init() {
		armSystem.setClawGrabbing(clawState);
		timer.reset();
	}

	@Override
	public void periodic() {

	}

	@Override
	public boolean completed() {
		return timer.seconds() > delayS;
	}

	@Override
	public void shutdown() {

	}
}
