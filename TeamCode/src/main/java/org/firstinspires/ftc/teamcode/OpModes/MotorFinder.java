package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MotorFinder extends LinearOpMode {
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
		waitForStart();
		while (opModeIsActive()) {
			BackLeft.setPower(-gamepad1.left_stick_y);
			BackRight.setPower(-gamepad1.right_stick_y);
			if (gamepad1.triangle) {
				aBreak.setPosition(0.7);
			}
			if (gamepad1.square) {
				aBreak.setPosition(0.5);
			}
		}
	}
}
