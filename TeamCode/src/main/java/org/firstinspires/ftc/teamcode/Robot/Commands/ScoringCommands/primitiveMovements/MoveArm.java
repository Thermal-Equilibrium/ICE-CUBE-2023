package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;

public class MoveArm extends Command {
	double delayS = 0.20;

	Turret turret;
	Turret.ArmStates armStates;

	ElapsedTime timer = new ElapsedTime();

	public MoveArm(Turret turret, Turret.ArmStates armStates) {
		this.turret = turret;
		this.armStates = armStates;
	}

	@Override
	public void init() {
		turret.setArm(armStates);
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
