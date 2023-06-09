package org.firstinspires.ftc.teamcode.Robot.Subsystems;


import android.util.Size;

import com.ThermalEquilibrium.homeostasis.Utils.SizedStack;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Math.Geometry.Twist2d;
import org.firstinspires.ftc.teamcode.RR_quickstart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utils.ExtraUtils;

import java.util.Objects;
import java.util.Vector;

public class Drivetrain extends Subsystem {
	protected HardwareMap hwMap;

	protected double leftPower = 0;
	protected double rightPower = 0;
	public SampleMecanumDrive drive;

	ElapsedTime integration_timer = new ElapsedTime();

	Pose2d imu_pose_estimate = new Pose2d();
	Vector2d imu_pose_estimate_vec = new Vector2d();
	Vector2d imu_velocity_estimate = new Vector2d();

	boolean has_begun_integration = false;

	Vector2d hubPosition_m = new Vector2d(0,-metersFomInches(2.5));


	SizedStack<Double> stoppedAccelerationsY = new SizedStack<>(60);
	SizedStack<Double> stoppedAccelerationsX = new SizedStack<>(60);

	double average_biasX = 0;
	double average_biasY = 0;

	@Override
	public void initAuto(HardwareMap hwMap) {
		this.hwMap = hwMap;
		drive = new SampleMecanumDrive(hwMap);
		imu_pose_estimate = drive.getPoseEstimate();
		imu_pose_estimate_vec = imu_pose_estimate.vec();
		integration_timer.reset();
		stoppedAccelerationsX.push(0.0);
		stoppedAccelerationsY.push(0.0);
	}

	@Override
	public void periodic() {

		if (!has_begun_integration) {
			integration_timer.reset();
			has_begun_integration = true;
		}
		drive.update();

		double dt = integration_timer.seconds();
		integration_timer.reset();
		Vector2d acceleration = getRobotRelativeAcceleration();

		double ang_vel_rad = drive.getExternalHeadingVelocity();
		Vector2d hub_acceleration_centripetal = hubPosition_m.times(ang_vel_rad * ang_vel_rad);

		hub_acceleration_centripetal = new Vector2d(
				inchesFromMeters(hub_acceleration_centripetal.getX()),
				inchesFromMeters(hub_acceleration_centripetal.getY())
		);

		acceleration = acceleration.minus(hub_acceleration_centripetal);


		if (robotIsLikelyStill()) {
			imu_velocity_estimate = new Vector2d();
			stoppedAccelerationsY.push(acceleration.getY());
			stoppedAccelerationsX.push(acceleration.getX());

			average_biasX = 0;
			average_biasY = 0;
			for (double accel : stoppedAccelerationsX) {
				average_biasX += accel;
			}
			for (double accel : stoppedAccelerationsY) {
				average_biasY += accel;
			}
			average_biasX = average_biasX / stoppedAccelerationsX.size();
			average_biasY = average_biasY / stoppedAccelerationsY.size();

		}

		acceleration = acceleration.plus(new Vector2d(-average_biasX,-average_biasY));
		acceleration = acceleration.rotated(drive.getPoseEstimate().getHeading());

		Vector2d velocityDelta = acceleration.times(dt);

		imu_velocity_estimate = imu_velocity_estimate.plus(velocityDelta);
		Vector2d positionDelta = imu_velocity_estimate.times(dt);
		imu_pose_estimate_vec = imu_pose_estimate_vec.plus(positionDelta);
		imu_pose_estimate = new Pose2d(imu_pose_estimate_vec,drive.getPoseEstimate().getHeading());




		Dashboard.packet.put("x acceleration",acceleration.getX());
		Dashboard.packet.put("y acceleration",acceleration.getY());
		Dashboard.packet.put("imu x position",imu_pose_estimate.getX());
		Dashboard.packet.put("imu y position",imu_pose_estimate.getY());
		Dashboard.packet.put("average accel bias x", average_biasX);
		Dashboard.packet.put("average accel bias y", average_biasY);
		ExtraUtils.drawRobotTarget(imu_pose_estimate,Dashboard.packet);

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

	public double getLeftPower() {
		return leftPower;
	}

	public double getRightPower() {
		return rightPower;
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

	protected Acceleration getAcceleration() {
		return drive.getImu().getLinearAcceleration();
	}

	public Vector2d getRobotRelativeAcceleration() {
		Acceleration acceleration = getAcceleration();
		return new Vector2d(inchesFromMeters(-acceleration.yAccel),inchesFromMeters(acceleration.zAccel));
	}

	public Vector2d getFieldRelativeAcceleration() {
		return getRobotRelativeAcceleration().rotated(getPose().getHeading());
	}


	public void setTrajectoryTracking(boolean shouldTrajectoryTrackNotStablilize) {
		drive.isHoldingPosition = !shouldTrajectoryTrackNotStablilize;
	}

	public void setHoldingPose(Pose2d pose) {
		drive.holdingPose = pose;
	}



	public double inchesFromMeters(double meters) {
		return meters * 39.370079;
	}

	public double metersFomInches(double inches) {
		return inches * 0.0254;
	}

	public void setPIDMode(boolean trajectory) {
		this.drive.setCoefficients(trajectory);
	}

	public boolean robotIsLikelyStill() {
		try {
			return Objects.requireNonNull(drive.getPoseVelocity()).vec().norm() < 0.0001;
		} catch (NullPointerException e) {
			return false;
		}
	}

}
