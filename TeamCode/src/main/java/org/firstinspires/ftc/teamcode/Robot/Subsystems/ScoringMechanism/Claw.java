package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;

@Config
public class Claw extends Subsystem {


	Servo claw_servo;
	public static double CLAW_OPEN = 0.45;
	public static double CLAW_CLOSED = 0.20;
	public static double CLAW_TRANSFER_SAFE = 0.335;

	public static double INTENTIONAL_DROP_DELAY = 1000.0;
	ElapsedTime intentionalDropTimer = new ElapsedTime();
	public boolean is_intentionally_waiting = false;

	@Override
	public void initAuto(HardwareMap hwMap) {
		claw_servo = hwMap.get(Servo.class,"servoClaw");
		claw_servo.setPosition(CLAW_CLOSED);
	}

	@Override
	public void initTeleop(HardwareMap hwMap) {
		claw_servo = hwMap.get(Servo.class,"servoClaw");
		claw_servo.setPosition(CLAW_OPEN);
	}

	@Override
	public void periodic() {
		if(is_intentionally_waiting && intentionalDropTimer.milliseconds() >= INTENTIONAL_DROP_DELAY) {
			is_intentionally_waiting = false;
		}
	}

	public void setPosition(double position) {
		claw_servo.setPosition(position);
	}

	@Override
	public void shutdown() {

	}

	public void startIntentionalDrop() {
		Log.i("CLAW", "Starting Intentional Drop");
		intentionalDropTimer.reset();
		is_intentionally_waiting = true;
	}


}
