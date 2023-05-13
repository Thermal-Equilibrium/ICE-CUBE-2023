package org.firstinspires.ftc.teamcode.Robot.Commands.SafetyAlgorithms;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RoadrunnerTrajectoryFollower;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;

import java.util.function.BooleanSupplier;

public class FollowIfNeeded extends RoadrunnerTrajectoryFollower {

	protected BooleanSupplier externalStop;

	public FollowIfNeeded(Robot robot, Trajectory traj, Dashboard dashboard, BooleanSupplier externalStop) {
		super(robot, traj, dashboard);
		this.externalStop = externalStop;
	}

	@Override
	public void init() {

		if (!externalStop.getAsBoolean()) {
			robot.drivetrain.followTrajectory(traj);
		}

	}

	@Override
	public boolean completed() {
		return !robot.drivetrain.isBusy() || externalStop.getAsBoolean();
	}
}
