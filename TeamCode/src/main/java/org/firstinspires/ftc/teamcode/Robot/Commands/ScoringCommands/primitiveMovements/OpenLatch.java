package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;

public class OpenLatch extends Command {

	Turret turret;
	boolean hasFinished = false;


	public OpenLatch(Turret turret) {
		this.turret = turret;
	}
	@Override
	public void init() {
		turret.open_latch();
	}

	@Override
	public void periodic() {
		turret.open_latch();
		hasFinished = true;
	}

	@Override
	public boolean completed() {
		return hasFinished;
	}

	@Override
	public void shutdown() {

	}
}