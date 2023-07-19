package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

@Config
public class Rotate extends Subsystem {

	public static double ROTATE_DEPOSIT = 0.71;
	public static double ROTATE_PICKUP = 0.155;


	Servo rotator;



	@Override
	public void initAuto(HardwareMap hwMap) {
		rotator = hwMap.get(Servo.class, "servoRotate");
		rotator.setPosition(ROTATE_PICKUP);
	}

	@Override
	public void periodic() {

	}

	@Override
	public void shutdown() {

	}

	public void setPosition(double position) {
		this.rotator.setPosition(position);
	}
}
