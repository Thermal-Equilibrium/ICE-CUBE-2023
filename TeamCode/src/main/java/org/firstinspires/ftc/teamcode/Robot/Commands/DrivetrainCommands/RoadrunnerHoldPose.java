package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;

public class RoadrunnerHoldPose extends Command {
	private final Robot robot;


	public RoadrunnerHoldPose(Robot robot, Pose2d pose) {
		super(robot.drivetrain);
		this.robot = robot;
		this.robot.drivetrain.setTrajectoryTracking(false);
		this.robot.drivetrain.setHoldingPose(pose);
	}


	@Override
	public void init() {
		this.robot.drivetrain.setPIDMode(false);
	}

	@Override
	public void periodic() {

	}

	@Override
	public boolean completed() {
		return true;
	}

	@Override
	public void shutdown() {
		this.robot.drivetrain.setPIDMode(true);
	}

}
