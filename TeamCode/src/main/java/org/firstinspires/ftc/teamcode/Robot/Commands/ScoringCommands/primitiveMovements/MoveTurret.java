package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;

public class MoveTurret extends Command {
	double delayS = 0.5;

	Turret turret;
	Turret.TurretStates turretStates;

	ElapsedTime timer = new ElapsedTime();

	public MoveTurret (Turret turret, Turret.TurretStates turretStates) {
		this.turret = turret;
		this.turretStates = turretStates;
	}

	@Override
	public void init() {
		turret.setTurret(turretStates);
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
