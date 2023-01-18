package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import static org.firstinspires.ftc.teamcode.RR_quickstart.trajectorysequence.TrajectorySequenceRunner.POSE_HISTORY_LIMIT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.RR_quickstart.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;

import java.util.LinkedList;

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
	}

}
