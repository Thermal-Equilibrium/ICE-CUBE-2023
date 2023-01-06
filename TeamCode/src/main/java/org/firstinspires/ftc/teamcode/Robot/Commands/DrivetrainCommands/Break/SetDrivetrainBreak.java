package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.Break;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;

public class SetDrivetrainBreak extends Command {

	Drivetrain drivetrain;
	Drivetrain.BreakStates breakState;
	boolean isComplete = false;

	public SetDrivetrainBreak(Drivetrain drivetrain, Drivetrain.BreakStates breakState) {
		super(drivetrain);
		this.drivetrain = drivetrain;
		this.breakState = breakState;
	}

	@Override
	public void init() {
		isComplete = true;
		drivetrain.setBreakState(breakState);
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
