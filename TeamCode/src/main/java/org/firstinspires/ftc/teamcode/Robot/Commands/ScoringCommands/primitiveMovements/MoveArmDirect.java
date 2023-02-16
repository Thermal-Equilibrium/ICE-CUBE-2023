package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;

public class MoveArmDirect extends Command {
	double delayS = 0.15;

	Turret turret;
	double position;

	ElapsedTime timer = new ElapsedTime();

	public MoveArmDirect(Turret turret, double position) {
		this.turret = turret;
		this.position = position;
	}

	@Override
	public void init() {
		turret.setArmDirect(position);
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
