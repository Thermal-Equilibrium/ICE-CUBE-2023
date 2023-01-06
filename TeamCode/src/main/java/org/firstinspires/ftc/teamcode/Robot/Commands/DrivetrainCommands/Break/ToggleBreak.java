package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.Break;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;

public class ToggleBreak extends Command {
	Drivetrain drivetrain;
	boolean isComplete = false;

	public ToggleBreak(Drivetrain drivetrain) {
		super(drivetrain);
		this.drivetrain = drivetrain;
	}

	@Override
	public void init() {
		isComplete = true;
		drivetrain.toggleBreakState();
	}

	@Override
	public void periodic() {

	}

	@Override
	public boolean completed() {
		return isComplete;
	}

	@Override
	public void shutdown() {

	}

}
