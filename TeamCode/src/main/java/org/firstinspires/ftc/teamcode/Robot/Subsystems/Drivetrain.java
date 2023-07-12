package org.firstinspires.ftc.teamcode.Robot.Subsystems;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.RR_quickstart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utils.PoseStorage;

public class Drivetrain extends Subsystem {
	protected HardwareMap hwMap;

	public SampleMecanumDrive drive;

	@Override
	public void initAuto(HardwareMap hwMap) {
		this.hwMap = hwMap;
		drive = new SampleMecanumDrive(hwMap);
	}

	@Override
	public void initTeleop(HardwareMap hwMap) {
		initAuto(hwMap);
		drive.setPoseEstimate(PoseStorage.pose_storage);
	}


	@Override
	public void periodic() {
		drive.update();
		PoseStorage.pose_storage = drive.getPoseEstimate();
	}


	public void robotRelative(Pose2d powers) {
		drive.setWeightedDrivePower(powers);
	}

	public void fieldRelative(Pose2d powers) {
		Vector2d vec = new Vector2d(powers.getX(), -powers.getY());

		vec = vec.rotated(drive.getPoseEstimate().getHeading());
		powers = new Pose2d(vec.getX(), -vec.getY(), powers.getHeading());
		robotRelative(powers);
	}

	public void followTrajectory(Trajectory traj) {
		drive.followTrajectoryAsync(traj);
	}

	@Override
	public void shutdown() {
		drive.setMotorPowers(0, 0, 0, 0);
	}


	public Pose2d getPose() {
		return drive.getPoseEstimate();
	}

	public void setPose(Pose2d pose) {
		drive.setPoseEstimate(pose);
	}

	public boolean isBusy() {
		return drive.isBusy();
	}

	public SampleMecanumDrive getBuilder() {
		return drive;
	}

	public Pose2d getVelocity() {
		return drive.getPoseVelocity();
	}


	public void setTrajectoryTracking(boolean shouldTrajectoryTrackNotStablilize) {
		drive.isHoldingPosition = !shouldTrajectoryTrackNotStablilize;
	}

	public void setHoldingPose(Pose2d pose) {
		drive.holdingPose = pose;
	}



	public void setPIDMode(boolean trajectory) {
		this.drive.setCoefficients(trajectory);
	}

}
