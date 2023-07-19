package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Claw;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Flip;

public class MoveFlip extends Command {

	double position;
	ElapsedTime timer = new ElapsedTime();
	double duration = 0.15;
	Flip flip;

	public MoveFlip(Flip flip, double position) {
		this.flip = flip;
		this.position = position;
	}

	@Override
	public void init() {
		timer.reset();
		this.flip.setPosition(position);
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
