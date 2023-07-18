package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
@Disabled
public class DrivebaseTest extends LinearOpMode {

    public static double POWER_STOP = 0.0;
    public static double POWER_SLOW = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        double power = 0.0;

        DcMotor motorFR = hardwareMap.get(DcMotor.class, "FrontRight");
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor motorFL = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.triangle) {
                power = POWER_SLOW;
            } else if (gamepad1.cross) {
                power = -POWER_SLOW;
            } else {
                power = POWER_STOP;
            }

            motorFL.setPower(power);
        }
    }
}