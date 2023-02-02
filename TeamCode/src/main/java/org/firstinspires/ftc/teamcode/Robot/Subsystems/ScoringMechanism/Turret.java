package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Math.Kinematics.Intake3DKinematics;

public class Turret extends Subsystem {

	public double armDown = 0.08;

	MainScoringMechanism.MechanismStates state = MainScoringMechanism.MechanismStates.BEGIN;

	ServoImplEx turret;
	Servo arm1;
	Servo claw;
	double clawTransferPosition = 0.34;
	double armSafe = 0.4;
	private static final double turretTransfer = Math.toRadians(0);
	private static final double MIN_RAW_SERVO_ANGLE = 0;
	private static final double MAX_RAW_SERVO_ANGLE = 1;

	public static final double MIN_SERVO_RADIANS = Math.toRadians(15);
	public static final double MAX_SERVO_RADIANS = Math.toRadians(345);

	double currentFreeStateValue = 0;
	Servo latch;

	double latch_open = 0.48;
	double latch_closed = 0.2;

	@Override
	public void initAuto(HardwareMap hwMap) {

		turret = hwMap.get(ServoImplEx.class, "turret");
//		turret.setPwmEnable();
//		turret.setPwmRange(new PwmControl.PwmRange(500,2500));

		turret.setPosition(turretTransfer);

		arm1 = hwMap.get(Servo.class,"arm");
		arm1.setDirection(Servo.Direction.REVERSE);
		arm1.setPosition(armSafe);
		claw = hwMap.get(Servo.class, "claw");
		claw.setDirection(Servo.Direction.FORWARD);
		claw.setPosition(0.5);
		latch = hwMap.get(Servo.class,"latch");
		latch.setPosition(latch_closed);

	}

	@Override
	public void initTeleop(HardwareMap hwMap) {
		turret = hwMap.get(ServoImplEx.class, "turret");
		turret.setPosition(turretTransfer);

		arm1 = hwMap.get(Servo.class,"arm");
		arm1.setDirection(Servo.Direction.REVERSE);
		arm1.setPosition(armSafe);
		claw = hwMap.get(Servo.class, "claw");
		claw.setDirection(Servo.Direction.FORWARD);
		claw.setPosition(0.5);
		latch = hwMap.get(Servo.class,"latch");
		latch.setPosition(latch_open);
	}

	@Override
	public void periodic() {

	}

	@Override
	public void shutdown() {

	}
	public void setTurret(TurretStates turretStates) {
		switch (turretStates) {
			case TRANSFER:
				setBasedTurretPosition(turretTransfer);
				break;
			case Slight_RIGHT:
				setBasedTurretPosition(Math.toRadians(30));
				break;
			case FAR_RIGHT:
				setBasedTurretPosition(Math.toRadians(60));
				break;
			case Slight_RIGHT_AUTO:
				setBasedTurretPosition(Math.toRadians(35));
				break;
			case Slight_LEFT:
				setBasedTurretPosition(Math.toRadians(-30));
				break;
			case FAR_LEFT:
				setBasedTurretPosition(Math.toRadians(-60));
				break;
		}
	}

	public void setRawTurretPosition(double position) {
		position = Range.clip(position, MIN_RAW_SERVO_ANGLE, MAX_RAW_SERVO_ANGLE);
		turret.setPosition(position);
	}
	public double getRawTurretPosition() {
		return turret.getPosition();
	}

	public void setBasedTurretPosition(double radians) {
		radians = Range.clip(radians, MIN_SERVO_RADIANS, MAX_SERVO_RADIANS);
		setRawTurretPosition(radiansToServo(radians));
	}
	public double getBasedTurretPosition() {
		return servoToRadians(getRawTurretPosition());
	}

	private double radiansToServo(double radians) {
		return Range.scale(radians, MIN_SERVO_RADIANS, MAX_SERVO_RADIANS, MIN_RAW_SERVO_ANGLE, MAX_RAW_SERVO_ANGLE);
	}

	private double servoToRadians(double servoAngle) {
		return Range.scale(servoAngle, MIN_RAW_SERVO_ANGLE, MAX_RAW_SERVO_ANGLE, MIN_SERVO_RADIANS, MAX_SERVO_RADIANS);
	}
	public enum TurretStates {
		TRANSFER,
		Slight_LEFT,
		Slight_RIGHT,
		Slight_RIGHT_AUTO,
		FAR_LEFT, // me
		FAR_RIGHT
	}

	public void setClawGrabbing(ClawStates clawState) {
		// TODO: Maybe add a different state for normally dropping cone from claw and dropping the cone in the outtake?
		switch (clawState) {
			case Open:
				// TODO: tune this values
				claw.setPosition(0.5);
				break;
			case Transfer:
				// TODO: tune this values
				claw.setPosition(clawTransferPosition);
				break;
			case Closed:
				// TODO: tune this values
				claw.setPosition(0);
		}
	}

	public void setArm(ArmStates armStates) {
		switch (armStates) {
			case TRANSFER:
				arm1.setPosition(0.19);
				break;
			case TRANSFER_SAFE:
				arm1.setPosition(armSafe);
				break;
			case DOWN:
				arm1.setPosition(armDown);
				break;
			case LOW_SCORING:
				arm1.setPosition(0.4);
			case FREE_STATE:
				arm1.setPosition(currentFreeStateValue);
				break;
		}
	}

	public void setArmDirect(double position) {
		currentFreeStateValue = position;
		setArm(ArmStates.FREE_STATE);
	}

	public void setArmPositionSync(double position) {
		// TODO: Adjust the min and max here to appropriate soft stops
		position = Range.clip(position,-1,1);
		arm1.setPosition(position);
//		arm2.setPosition(1 - position); uncomment once second servo is connected
	}

	// TODO: Maybe don't average? if we only use 1 servo lol
	public double getArmPosition() {

		return arm1.getPosition();
	}
	public enum ClawStates {
		Open,
		Transfer,
		Closed
	}

	public enum ArmStates {
		TRANSFER,
		TRANSFER_SAFE,
		DOWN,
		LOW_SCORING,
		FREE_STATE
	}

	public void close_latch() {
		latch.setPosition(latch_closed);
	}

	public void open_latch() {
		latch.setPosition(latch_open);
	}

	public void setCurrentFreeStateValue(double currentFreeStateValue) {
		this.currentFreeStateValue = currentFreeStateValue;
	}

	public void setArmKinematicPosition(double y, double z) {
		setBasedTurretPosition(Intake3DKinematics.getYawAngleToTarget(y, z));
		setArmDirect(Intake3DKinematics.getPitchAngleToTarget(z));

	}
}
