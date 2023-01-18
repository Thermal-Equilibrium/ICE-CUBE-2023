package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;

public class Turret extends Subsystem {

	public double armDown = 0.08;

	MainScoringMechanism.MechanismStates state = MainScoringMechanism.MechanismStates.BEGIN;

	Servo turret;
	Servo arm1;
	Servo claw;
	double clawTransferPosition = 0.34;
	double armSafe = 0.4;
	double turretTransfer = 0.51889;
	double currentFreeStateValue = 0;



	@Override
	public void initAuto(HardwareMap hwMap) {

		turret = hwMap.get(Servo.class, "turret");
		turret.setPosition(turretTransfer);

		arm1 = hwMap.get(Servo.class,"arm");
		arm1.setDirection(Servo.Direction.REVERSE);
		arm1.setPosition(armSafe);
		claw = hwMap.get(Servo.class, "claw");
		claw.setDirection(Servo.Direction.FORWARD);
		claw.setPosition(0.5);
	}

	@Override
	public void periodic() {

	}

	@Override
	public void shutdown() {

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

	public void setTurret(TurretStates turretStates) {
		switch (turretStates) {
			case TRANSFER:
				setTurretPositionSync(turretTransfer);
				break;
			case Slight_LEFT:
				setTurretPositionSync(1);
				break;
			case Slight_RIGHT:
				setTurretPositionSync(0);
				break;
			case Slight_RIGHT_AUTO:
				setTurretPositionSync(0.0169);
				break;
			case FAR_LEFT:
				setTurretPositionSync(0.9);
				break;
			case FAR_RIGHT:
				setTurretPositionSync(0.1);
				break;
		}
	}

	public void setTurretPositionSync(double position) {
		position = Range.clip(position,-1,1);
		turret.setPosition(position);
		Dashboard.packet.put("RECIEVES POS", position);
	}

	public double getTurretPosition() {
		return turret.getPosition();
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

	public enum TurretStates {
		TRANSFER,
		Slight_LEFT,
		Slight_RIGHT,
		Slight_RIGHT_AUTO,
		FAR_LEFT, // me
		FAR_RIGHT
	}

	public enum ArmStates {
		TRANSFER,
		TRANSFER_SAFE,
		DOWN,
		LOW_SCORING,
		FREE_STATE
	}


	public void setCurrentFreeStateValue(double currentFreeStateValue) {
		this.currentFreeStateValue = currentFreeStateValue;
	}
}
