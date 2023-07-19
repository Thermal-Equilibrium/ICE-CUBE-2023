package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

@Config
public class Claw extends Subsystem {


	Servo claw_servo;
	public static double CLAW_OPEN = 0.45;
	public static double CLAW_CLOSED = 0.22;
	public static double CLAW_TRANSFER_SAFE = 0.335;



	@Override
	public void initAuto(HardwareMap hwMap) {
		claw_servo = hwMap.get(Servo.class,"servoClaw");
		claw_servo.setPosition(CLAW_OPEN);
	}

	@Override
	public void periodic() {

	}

	public void setPosition(double position) {
		claw_servo.setPosition(position);
	}

	@Override
	public void shutdown() {

	}


}
