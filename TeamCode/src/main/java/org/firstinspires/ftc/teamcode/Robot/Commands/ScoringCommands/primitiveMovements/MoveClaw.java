package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Claw;

public class MoveClaw extends Command {

	double position;
	ElapsedTime timer = new ElapsedTime();
	double duration = 0.15;
	Claw claw;

	public MoveClaw(Claw claw, double position) {
		this.claw = claw;
		this.position = position;
	}


	@Override
	public void init() {
		timer.reset();
		claw.setPosition(position);
	}

	@Override
	public void periodic() {

	}

	@Override
	public boolean completed() {
		return timer.seconds() > duration;
	}

	@Override
	public void shutdown() {

	}
}
