package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.Brake;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;

public class SetDrivetrainBrake extends Command {

	Drivetrain drivetrain;
	Drivetrain.BrakeStates brakeState;
	boolean isComplete = false;

	public SetDrivetrainBrake(Drivetrain drivetrain, Drivetrain.BrakeStates brakeState) {
		super(drivetrain);
		this.drivetrain = drivetrain;
		this.brakeState = brakeState;
	}

	@Override
	public void init() {
		isComplete = true;
		drivetrain.setBrakeState(brakeState);
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
