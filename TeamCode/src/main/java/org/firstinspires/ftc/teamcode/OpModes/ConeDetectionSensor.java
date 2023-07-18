package org.firstinspires.ftc.teamcode.OpModes;

import static java.lang.Double.isNaN;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utils.BallerFilter;

import java.util.ArrayList;


@TeleOp
public class ConeDetectionSensor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

//        ColorSensor colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        DistanceSensor sensorLeft = hardwareMap.get(DistanceSensor.class, "colorLeft");

//        ColorSensor colorRight = hardwareMap.get(ColorSensor.class, "colorRight");
        DistanceSensor sensorRight = hardwareMap.get(DistanceSensor.class, "colorRight");

        waitForStart();
        final int sampleSize = 20;
        double distLeft;
        double distRight;
        ArrayList<Double> leftData = new ArrayList<>();
        double leftFiltered = 100.0;
        ArrayList<Double> rightData = new ArrayList<>();
        double rightFiltered = 100.0;
        boolean canGrab = false;

        while (opModeIsActive()) {
            distLeft = sensorLeft.getDistance(DistanceUnit.CM);
            distRight = sensorRight.getDistance(DistanceUnit.CM);

            leftData.add(distLeft);
            rightData.add(distRight);

            if(leftData.size() > sampleSize) {
                BallerFilter ballerFilter = new BallerFilter(leftData.toArray(new Double[leftData.size()]));
                leftFiltered = ballerFilter.computeResult();
                leftData.clear();
            }

            if(rightData.size() > sampleSize) {
                BallerFilter ballerFilter = new BallerFilter(rightData.toArray(new Double[rightData.size()]));
                rightFiltered = ballerFilter.computeResult();
                rightData.clear();
            }

            if(leftFiltered <= 10.0 && rightFiltered <= 15.0) {
                canGrab = true;
            } else {
                canGrab = false;
            }

            telemetry.addData("Can Grab", canGrab);
            telemetry.addData("Filtered Left", leftFiltered);
            telemetry.addData("Filtered Right", rightFiltered);

            telemetry.addData("Dist Left", sensorLeft.getDistance(DistanceUnit.CM));
//            telemetry.addData("Red Left", colorLeft.red());
//            telemetry.addData("Green Left", colorLeft.green());
//            telemetry.addData("Blue Left", colorLeft.blue());

            telemetry.addData("Dist Right", sensorRight.getDistance(DistanceUnit.CM));
//            telemetry.addData("Red Right", colorRight.red());
//            telemetry.addData("Green Right", colorRight.green());
//            telemetry.addData("Blue Right", colorRight.blue());
            telemetry.update();
        }
    }
}