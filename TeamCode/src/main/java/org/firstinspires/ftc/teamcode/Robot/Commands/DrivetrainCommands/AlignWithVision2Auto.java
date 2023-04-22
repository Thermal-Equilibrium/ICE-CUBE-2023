package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DistanceSensor;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;

public class AlignWithVision2Auto extends Command {
    public static PIDCoefficients controllerCoefficientsDistance = new PIDCoefficients(0.1, 0, 0.05);
    public static double referenceDistanceSensor = 9; // distance in inches away from the pole
    protected BasicPID controller = new BasicPID(controllerCoefficientsDistance);
    Drivetrain drivetrain;
    DistanceSensor distanceSensor;
    double error = 10;
    double error_tolerance = 0.5;


    public AlignWithVision2Auto(Drivetrain drivetrain, DistanceSensor distanceSensor) {
        this.drivetrain = drivetrain;
        this.distanceSensor = distanceSensor;
    }

    @Override
    public void init() {

    }

    @Override
    public void periodic() {
        error = referenceDistanceSensor - distanceSensor.getDistance_in();
        double power = controller.calculate(referenceDistanceSensor, distanceSensor.getDistance_in()) + Math.signum(error) * 0.06;
        drivetrain.robotRelative(new Pose2d(-power, 0, 0));
    }

    @Override
    public boolean completed() {
        return Math.abs(error) < error_tolerance || distanceSensor.getDistance_in() > 24;
    }

    @Override
    public void shutdown() {
        drivetrain.robotRelative(new Pose2d(0, 0, 0));
    }

}
