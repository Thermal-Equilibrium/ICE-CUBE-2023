package org.firstinspires.ftc.teamcode.Purepursuit.Utils;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.MotionConstraint;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;

public class ProfiledServo extends Subsystem {
	public Servo servo_right;
	public Servo servo_left;
	public MotionProfile profile_m;
	public MotionConstraint forwardConstraint;
	public MotionConstraint backwardContraint;
	public ElapsedTime timer = new ElapsedTime();
	protected double endPosition;
	protected double previousEndPosition;
	protected double currentPosition;
	protected String name;

	public ProfiledServo(HardwareMap hwmap, String name1, String name2, double Forwardvelo, double Forwardaccel, double BackwardVelo, double BackwardAccel, double initialPosition) {
		servo_left = hwmap.get(Servo.class, name1);
		servo_right = hwmap.get(Servo.class, name2);
		this.name = name1 + " " + name2 + " ";
		this.endPosition = initialPosition;
		this.currentPosition = initialPosition;
		this.previousEndPosition = initialPosition + 100; // just guarantee that they are not equal
		this.forwardConstraint = new MotionConstraint(Forwardvelo, Forwardaccel, Forwardaccel);
		this.backwardContraint = new MotionConstraint(BackwardVelo, BackwardAccel, BackwardAccel);
		setPositionsSynced(initialPosition);
	}

	protected void regenerate_profile() {
		if (endPosition > previousEndPosition) {
			profile_m = MotionProfileGenerator.generateSimpleMotionProfile(
					new MotionState(currentPosition, 0, 0),
					new MotionState(endPosition, 0, 0),
					forwardConstraint.max_velocity,
					forwardConstraint.max_acceleration,
					75
			);
		} else {
			profile_m = MotionProfileGenerator.generateSimpleMotionProfile(
					new MotionState(currentPosition, 0, 0),
					new MotionState(endPosition, 0, 0),
					backwardContraint.max_velocity,
					backwardContraint.max_acceleration,
					75
			);
		}

		timer.reset();
	}

	@Override
	public void initAuto(HardwareMap hwMap) {

	}

	@Override
	public void periodic() {
		if (endPosition != previousEndPosition) {
			regenerate_profile();
		}
		previousEndPosition = endPosition;
		double current_target = profile_m.get(timer.seconds()).getX();
		setPositionsSynced(current_target);
		Dashboard.packet.put(name + "position: ", current_target);
	}

	public boolean isBusy() {
		return timer.seconds() < profile_m.duration();
	}

	@Override
	public void shutdown() {

	}

	public void setPosition(double endPosition) {
		this.endPosition = endPosition;
	}

	protected void setPositionsSynced(double pos) {
		servo_left.setPosition(1 - pos);
		servo_right.setPosition(pos);
	}
}
