package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.UltimateGoalMoment;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.returnToUltimateGoal.MotionProfiledADRCPoseStabilizationController;

public class UGLqrPoseStabilization extends Command {


	Drivetrain drivetrain;
	Pose2d targetPose;
	MotionProfiledADRCPoseStabilizationController controller = new MotionProfiledADRCPoseStabilizationController(true);

	public UGLqrPoseStabilization(Drivetrain drivetrain, Pose2d targetPose) {
		this.drivetrain = drivetrain;
		this.targetPose = targetPose;
	}

	@Override
	public void init() {

	}

	@Override
	public void periodic() {
		Pose2d driveSignal = controller.goToPosition(targetPose,drivetrain.getPose());
		drivetrain.fieldRelative(driveSignal);
		Dashboard.packet.put("x power",driveSignal.getX());
		Dashboard.packet.put("y power",driveSignal.getY());
		Dashboard.packet.put("x error", controller.getPoseError().getX());
		Dashboard.packet.put("y error", controller.getPoseError().getY());
		Dashboard.packet.put("heading error", controller.getPoseError().getHeading());
		Dashboard.packet.put("error mag", controller.errorMag());
	}

	@Override
	public boolean completed() {
		return  (controller.errorMag() < 1.2
				&& controller.headingErrorMag() < Math.toRadians(2)
				&& controller.getErrorMagDeriv() < 1
				&& Math.abs(drivetrain.getVelocity().getHeading()) < Math.toRadians(3))
				|| (
				Math.abs(drivetrain.getVelocity().getHeading()) < Math.toRadians(0.5)
						&& controller.getErrorMagDeriv() < 0.2 && controller.errorMag() < 4 // might die for short paths
		);	}

	@Override
	public void shutdown() {
		drivetrain.robotRelative(new Pose2d(0,0,0));
	}
}
