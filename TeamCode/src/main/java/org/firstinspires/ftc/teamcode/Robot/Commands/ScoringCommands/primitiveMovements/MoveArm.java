package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.ArmSystem;

public class MoveArm extends Command {
	double delayS = 0.15;

	ArmSystem armSystem;
	ArmSystem.ArmStates armStates;

	ElapsedTime timer = new ElapsedTime();

	public MoveArm(ArmSystem armSystem, ArmSystem.ArmStates armStates) {
		this.armSystem = armSystem;
		this.armStates = armStates;
	}

	@Override
	public void init() {
		armSystem.setArm(armStates);
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
