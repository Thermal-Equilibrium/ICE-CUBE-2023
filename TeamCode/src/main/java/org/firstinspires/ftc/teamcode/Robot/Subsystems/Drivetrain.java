package org.firstinspires.ftc.teamcode.Robot.Subsystems;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RR_quickstart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class Drivetrain extends Subsystem {
	protected HardwareMap hwMap;

	protected double leftPower = 0;
	protected double rightPower = 0;
	SampleMecanumDrive drive;
	Servo aBreak;
	double breakActive = 0.5;
	double breakReleased = 0.7;
	BreakStates breakState = BreakStates.FREE;

	@Override
	public void initAuto(HardwareMap hwMap) {
		this.hwMap = hwMap;
		drive = new SampleMecanumDrive(hwMap);
		aBreak = hwMap.get(Servo.class, "break");
		aBreak.setPosition(breakReleased);
	}
	@Override
	public void periodic() {
		drive.update();
		switch (breakState) {
			case FREE:
				aBreak.setPosition(breakReleased);
				break;
			case ACTIVATED:
				aBreak.setPosition(breakActive);
				break;
		}

	}


	public void  robotRelative(Pose2d powers) {
		drive.setWeightedDrivePower(powers);
	}

	public void fieldRelative(Pose2d powers) {
		Vector2d vec = new Vector2d(powers.getX(),-powers.getY());

		vec = vec.rotated(drive.getPoseEstimate().getHeading());
		powers = new Pose2d(vec.getX(),-vec.getY(),powers.getHeading());
		robotRelative(powers);
	}

	public void followTrajectory(Trajectory traj) {
		drive.followTrajectoryAsync(traj);
	}

	@Override
	public void shutdown() {
		drive.setMotorPowers(0,0,0,0);
	}

	public double getLeftPower() {
		return leftPower;
	}

	public double getRightPower() {
		return rightPower;
	}

	public Pose2d getPose() {
		return drive.getPoseEstimate();
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

	public void setPose(Pose2d pose) {
		drive.setPoseEstimate(pose);
	}


	public void setBreakState(BreakStates breakState) {
		this.breakState = breakState;
	}

	public BreakStates getBreakState() {
		return breakState;
	}

	public void toggleBreakState() {
		if (breakState.equals(BreakStates.ACTIVATED)) {
			breakState = BreakStates.FREE;
		} else {
			breakState = BreakStates.ACTIVATED;
		}
	}

	public enum BreakStates {
		FREE,
		ACTIVATED
	}

	public void setTrajectoryTracking(boolean shouldTrajectoryTrackNotStablilize) {
		drive.isHoldingPosition = !shouldTrajectoryTrackNotStablilize;
	}

	public void setHoldingPose(Pose2d pose) {
		drive.holdingPose = pose;
	}


}
