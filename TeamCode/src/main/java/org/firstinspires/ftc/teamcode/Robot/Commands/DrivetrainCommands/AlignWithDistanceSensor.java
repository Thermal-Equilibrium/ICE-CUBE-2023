package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import static org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.AlignWithVision2Auto.controllerCoefficientsDistance;
import static org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.AlignWithVision2Auto.referenceDistanceSensor;

import android.os.Build;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DistanceSensor;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;

import java.util.function.BooleanSupplier;

public class AlignWithDistanceSensor extends Command {

	Drivetrain drivetrain;
	DistanceSensor distanceSensor;

	protected BasicPID controller = new BasicPID(controllerCoefficientsDistance);
	BooleanSupplier keepRunning;


	public AlignWithDistanceSensor(Drivetrain drivetrain, DistanceSensor distanceSensor, BooleanSupplier keepRunning) {
		this.drivetrain = drivetrain;
		this.distanceSensor = distanceSensor;
		this.keepRunning = keepRunning;
	}

	@Override
	public void init() {

	}

	@Override
	public void periodic() {

		double power = controller.calculate(referenceDistanceSensor, distanceSensor.getDistance_in());
		drivetrain.robotRelative(new Pose2d(-power,0,0));
	}

	@Override
	public boolean completed() {
		if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
			return !keepRunning.getAsBoolean();
		}
		return true;
	}

	@Override
	public void shutdown() {
		drivetrain.robotRelative(new Pose2d(0,0,0));
	}
}
