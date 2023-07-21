package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RR_quickstart.drive.SampleMecanumDrive;

import java.util.List;

@TeleOp
public class read_drivetrain_encoders extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            drive.update();
            System.out.println("wheel positions: " + drive.getWheelTicks());
        }
    }
}