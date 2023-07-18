package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp
@Disabled
public class SlidesTest extends LinearOpMode {

    public static double SLIDES_STOP = 0.0;
    public static double SLIDES_SLOW = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        double slidesPower = 0.0;

        DcMotor slidesRight = hardwareMap.get(DcMotor.class, "slidesRight");
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotor slidesLeft = hardwareMap.get(DcMotor.class, "slidesLeft");
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.triangle) {
                // Slides up
                slidesPower = SLIDES_SLOW;
            } else if (gamepad1.cross) {
                // Slides downj
                slidesPower = -SLIDES_SLOW;
            } else {
                slidesPower = SLIDES_STOP;
            }

            slidesLeft.setPower(slidesPower);
            slidesRight.setPower(slidesPower);
            telemetry.addData("slidesPower: ", slidesPower);
            telemetry.addData("slidesLeft: ", slidesLeft.getCurrentPosition());
            telemetry.addData("slidesRight: ", slidesRight.getCurrentPosition());
            telemetry.update();
        }
    }
}