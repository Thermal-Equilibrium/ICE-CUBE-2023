package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;

public class MoveTurretAsync extends MoveTurret{
	public MoveTurretAsync(Turret turret, Turret.TurretStates turretStates) {
		super(turret, turretStates);
	}
	@Override
	public boolean completed() {
		return true;
	}
}
