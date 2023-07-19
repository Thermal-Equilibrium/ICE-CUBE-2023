package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.java_websocket.handshake.ServerHandshakeBuilder;


@TeleOp
public class ServoCycle extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double outAngle = 1.0;
        double inAngle = 0.0;
        double smallAdjust = 0.01;
        double largeAdjust = 0.10;

        boolean prev_triangle = false;
        boolean prev_cross = false;
        boolean prev_square = false;
        boolean prev_circle = false;

        double rotatorPickup = 0.155;
        double rotatorDeposit = 0.71;

        double flipPickup = 0.22;
        double flipDeposit = 0.78;
        double flipPoleAlign = 0.66;
        double flipFolded = 0.38;

        double claw_open = 0.53;
        double claw_closed = 0.18;

        double rotatorPos = rotatorPickup;
        double flipPos = flipPickup;


//        Servo servo = hardwareMap.get(Servo.class, "servoFlip");
        Servo rotator = hardwareMap.get(Servo.class, "servoRotate");
        Servo flip = hardwareMap.get(Servo.class, "servoFlip");
        waitForStart();
        while (opModeIsActive()) {
            boolean triangle = gamepad1.triangle;
            boolean cross = gamepad1.cross;
            boolean square = gamepad1.square;
            boolean circle = gamepad1.circle;

            if (triangle && !prev_triangle) {
                flipPos += largeAdjust;
            }
            if (cross && !prev_cross) {
                flipPos -= largeAdjust;
            }
            if (square && !prev_square) {
               flipPos += smallAdjust;
            }
            if (circle && !prev_circle) {
                flipPos -= smallAdjust;
            }

            prev_circle = circle;
            prev_cross = cross;
            prev_triangle = triangle;
            prev_square = square;

            if (gamepad1.dpad_up) {
                rotatorPos = rotatorPickup;
                flipPos = flipPickup;
            } else if (gamepad1.dpad_down) {
                rotatorPos = rotatorDeposit;
                flipPos = flipDeposit;
            }

            rotator.setPosition(Range.clip(rotatorPos, rotatorPickup, rotatorDeposit));
            flip.setPosition(Range.clip(flipPos, flipPickup, flipDeposit));

            telemetry.addData("rotator angle: ", rotatorPos);
            telemetry.addData("flip angle: ", flipPos);
            telemetry.update();
        }
    }
}