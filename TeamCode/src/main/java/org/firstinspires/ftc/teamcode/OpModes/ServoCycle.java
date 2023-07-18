package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class ServoCycle extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double outAngle = 0.99;
        double inAngle = 0.01;

        Servo claw = hardwareMap.get(Servo.class, "servoClaw");
        waitForStart();
        double angle = outAngle;
        while (opModeIsActive()) {

            if (gamepad1.triangle) {
                angle = outAngle;
            }
            if (gamepad1.cross) {
                angle = inAngle;
            }

            claw.setPosition(angle);
            telemetry.addData("claw angle: ",angle);
            telemetry.update();
        }
    }
}