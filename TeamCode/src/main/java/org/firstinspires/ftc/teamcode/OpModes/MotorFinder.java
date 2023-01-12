package org.firstinspires.ftc.teamcode.OpModes;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.MotionConstraint;
import org.firstinspires.ftc.teamcode.Utils.ProfiledPID;

@TeleOp
public class MotorFinder extends LinearOpMode {


	double highPosition = 833;
	double setpoint = 150;
	PIDCoefficients coefficients = new PIDCoefficients(0.01,0,0);
	MotionConstraint upConstraint = new MotionConstraint(5000,5000,2000);
	MotionConstraint downConstraint = new MotionConstraint(5000,5000,2000);

	ProfiledPID pid = new ProfiledPID(upConstraint,downConstraint,coefficients);
	@Override
	public void runOpMode() throws InterruptedException {
		Servo aBreak = hardwareMap.get(Servo.class, "break");
		DcMotorEx rightHorizontal = hardwareMap.get(DcMotorEx.class, "rightHorizontal");
		rightHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
		DcMotorEx FrontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
		FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
		DcMotorEx FrontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
		FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		DcMotorEx leftHorizontal = hardwareMap.get(DcMotorEx.class, "leftHorizontal");
		leftHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		leftHorizontal.setDirection(DcMotorSimple.Direction.FORWARD);
		DcMotorEx vertical1 = hardwareMap.get(DcMotorEx.class, "vertical1");
		vertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		vertical1.setDirection(DcMotorSimple.Direction.FORWARD);
		DcMotorEx vertical2 = hardwareMap.get(DcMotorEx.class, "vertical2");
		vertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		vertical2.setDirection(DcMotorSimple.Direction.REVERSE);
		DcMotorEx BackLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
		BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		DcMotorEx BackRight = hardwareMap.get(DcMotorEx.class, "BackRight");
		BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		BackRight.setDirection(DcMotorSimple.Direction.FORWARD);
		Servo arm = hardwareMap.get(Servo.class,"arm");
		arm.setDirection(Servo.Direction.REVERSE);
		Servo claw = hardwareMap.get(Servo.class,"claw");
		Servo s2 = hardwareMap.get(Servo.class, "ch2");
		Servo turret = hardwareMap.get(Servo.class, "turret");
		waitForStart();
		double turretAngle = 0.51889;
		while (opModeIsActive()) {
//			turretAngle += -gamepad1.left_stick_y * 0.01;
//			setpoint -= gamepad1.right_stick_y;
//			turret.setPosition(turretAngle);
			//s1.setPosition(turretAngle);
			// s0.setPosition(0.5);
//			arm.setPosition(0.3);
			claw.setPosition(0.5);

			double slidePosition = (leftHorizontal.getCurrentPosition() + rightHorizontal.getCurrentPosition()) / 2.0;
			double power = 0;//pid.calculate(setpoint,slidePosition);

			leftHorizontal.setPower(power);
			rightHorizontal.setPower(power);
			telemetry.addData("target position",setpoint);
			telemetry.addData("turret angle",turretAngle);
			telemetry.update();
			if (gamepad1.triangle) {
				arm.setPosition(0.4);
			}
			if (gamepad1.square) {
				arm.setPosition(0.1978);
			}
		}
	}
}
