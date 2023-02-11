package org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DistanceSensor;

public class SetPoleContext extends Command {

	DistanceSensor distanceSensor;
	boolean hasFinished = false;

	public SetPoleContext(DistanceSensor distanceSensor) {
		super(distanceSensor);
		this.distanceSensor = distanceSensor;
	}

	@Override
	public void init() {
		distanceSensor.calculatePolePositionInContext();
		hasFinished = true;
	}

	@Override
	public void periodic() {

	}

	@Override
	public boolean completed() {
		return hasFinished;
	}

	@Override
	public void shutdown() {

	}
}
