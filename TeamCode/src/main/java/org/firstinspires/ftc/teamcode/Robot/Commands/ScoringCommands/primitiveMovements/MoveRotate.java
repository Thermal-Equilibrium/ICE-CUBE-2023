package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Flip;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Rotate;

public class MoveRotate extends Command {

	double position;
	ElapsedTime timer = new ElapsedTime();
	double duration = 0.15;
	Rotate rotate;

	public MoveRotate(Rotate rotate, double position) {
		this.rotate = rotate;
		this.position = position;
	}


	@Override
	public void init() {
		timer.reset();
		rotate.setPosition(this.position);
	}

	@Override
	public void periodic() {

	}

	@Override
	public boolean completed() {
		return timer.seconds() > this.duration;
	}

	@Override
	public void shutdown() {

	}
}
