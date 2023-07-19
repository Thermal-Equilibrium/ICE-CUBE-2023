package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;


@Config
public class Flip extends Subsystem {

	public static double FLIP_PICKUP = 0.24;
	public static double FLIP_DEPOSIT = 0.78;
	public static double FLIP_POLE_ALIGN = 0.6;
	public static double FLIP_FOLDED = 0.38;
	public boolean is_folded = false;

	Servo flip;


	@Override
	public void initAuto(HardwareMap hwMap) {
		flip = hwMap.get(Servo.class, "servoFlip");
		flip.setPosition(FLIP_PICKUP);
	}

	@Override
	public void periodic() {

	}

	@Override
	public void shutdown() {

	}


	public void setPosition(double position) {

		is_folded = position == FLIP_FOLDED;

		flip.setPosition(position);
	}

	public double getPosition() {
		return flip.getPosition();
	}

}
