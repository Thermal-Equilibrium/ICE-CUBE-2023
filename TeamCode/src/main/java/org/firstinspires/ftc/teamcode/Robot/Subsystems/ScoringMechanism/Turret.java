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

	@Override
	public void initAuto(HardwareMap hwMap) {

		turret = hwMap.get(Servo.class, "turret");
		arm1 = hwMap.get(Servo.class,"arm1");
		arm2 = hwMap.get(Servo.class,"arm2");
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

	public void setClawGrabbing(boolean grabbing) {
		// TODO: Maybe add a different state for normally dropping cone from claw and dropping the cone in the outtake?
		if (grabbing)
			claw.setPosition(0); // set to value that best grabs cone
		else
			claw.setPosition(1); // set to value that best drops cone, should be as small as possible so that it fits between the outtake slides
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
		arm2.setPosition(1 - position);
	}

	// TODO: Maybe don't average? if we only use 1 servo lol
	public double getArmPosition() {
		double arm1Position = arm1.getPosition();
		double arm2Position = 1 - arm2.getPosition();

		return (arm1Position + arm2Position) / 2;
	}

}
