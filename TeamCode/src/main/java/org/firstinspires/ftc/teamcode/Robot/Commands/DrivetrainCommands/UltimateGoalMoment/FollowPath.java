package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.UltimateGoalMoment;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.returnToUltimateGoal.MotionProfiledADRCPoseStabilizationController;

import java.util.Collections;
import java.util.LinkedList;


/**
 * follows a path of pose2d markers until the end.
 */
public class FollowPath extends Command {

	MotionProfiledADRCPoseStabilizationController controller = new MotionProfiledADRCPoseStabilizationController(true);
	LinkedList<Pose2d> positions = new LinkedList<>();
	Drivetrain drivetrain;
	Pose2d currentTarget;
	Pose2d power = new Pose2d();

	public FollowPath(Drivetrain drivetrain, Pose2d... pose2ds) {
		this.drivetrain = drivetrain;
		Collections.addAll(positions,pose2ds);
	}


	@Override
	public void init() {
		currentTarget = positions.removeFirst();
	}

	@Override
	public void periodic() {

		drivetrain.fieldRelative(power);
		if (isDoneWithPoint() && positions.size() >= 1) {
			currentTarget = positions.removeFirst();
			controller = new MotionProfiledADRCPoseStabilizationController(800);
		}
		power = controller.goToPosition(currentTarget, drivetrain.getPose());

	}

	@Override
	public boolean completed() {
		return positions.size() < 1 && isDoneWithPoint();
	}

	@Override
	public void shutdown() {
		drivetrain.robotRelative(new Pose2d());
	}

	protected boolean isDoneWithPoint() {
		return  (controller.errorMag() < 1.2
				&& controller.headingErrorMag() < Math.toRadians(2)
				&& controller.getErrorMagDeriv() < 1
				&& Math.abs(drivetrain.getVelocity().getHeading()) < Math.toRadians(3))
				|| (
				Math.abs(drivetrain.getVelocity().getHeading()) < Math.toRadians(3)
				&& controller.getErrorMagDeriv() < 0.2 && controller.errorMag() < 4 // might die for short paths
			);

	}

}
