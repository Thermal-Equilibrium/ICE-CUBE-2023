package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class Turret extends Subsystem {

	MainScoringMechanism.MechanismStates state = MainScoringMechanism.MechanismStates.BEGIN;

	Servo turretServo;
	Servo arm1;
	Servo arm2;

	@Override
	public void initAuto(HardwareMap hwMap) {

		turretServo = hwMap.get(Servo.class, "turret");
		arm1 = hwMap.get(Servo.class,"arm1");
		arm2 = hwMap.get(Servo.class,"arm2");
		arm2.setDirection(Servo.Direction.REVERSE);

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

	protected void setServoPositionSync(double position) {
		position = Range.clip(position,-1,1);
		arm1.setPosition(position);
		arm2.setPosition(1 - position);
	}

}
