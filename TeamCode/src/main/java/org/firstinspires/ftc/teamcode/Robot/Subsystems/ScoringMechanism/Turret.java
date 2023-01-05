package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class Turret extends Subsystem {



	MainScoringMechanism.MechanismStates state = MainScoringMechanism.MechanismStates.BEGIN;

	Servo turret;
	Servo arm1;
	Servo arm2;
	Servo claw;

	public enum ClawStates {
		Open,
		Transfer,
		Closed
	}

	@Override
	public void initAuto(HardwareMap hwMap) {

		turret = hwMap.get(Servo.class, "turret");
		arm1 = hwMap.get(Servo.class,"arm");
//		arm2 = hwMap.get(Servo.class,"arm2");
		arm2.setDirection(Servo.Direction.REVERSE);
		claw = hwMap.get(Servo.class, "claw");
	}

	@Override
	public void periodic() {

	}

	@Override
	public void shutdown() {

	}
	public void setState(MainScoringMechanism.MechanismStates state) {
		this.state = state;
	}

	public void setClawGrabbing(ClawStates clawState) {
		// TODO: Maybe add a different state for normally dropping cone from claw and dropping the cone in the outtake?
		switch (clawState) {
			case Open:
				// TODO: tune this values
				claw.setPosition(1);
				break;
			case Transfer:
				// TODO: tune this values
				claw.setPosition(0.5);
				break;
			case Closed:
				// TODO: tune this values
				claw.setPosition(0);
		}
	}

	public void setTurretPositionSync(double position) {
		position = Range.clip(position,-1,1);
		turret.setPosition(position);
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
		double arm1Position = arm1.getPosition();

		return arm1Position;

//		double arm2Position = 1 - arm2.getPosition();
//														uncomment once second servo is connected
//		return (arm1Position + arm2Position) / 2;
	}

}
