package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;

@Autonomous
public class LeftHighPole extends BaseAuto {

	final Pose2d START_POSE = new Pose2d(-31, -64.5, Math.toRadians(-90));
	final Pose2d SCORE_POSE_ZERO = new Pose2d(-25, -0, Math.toRadians(180 + 45));
	final Pose2d SCORE_POSE_ONE = shiftRobotRelative(SCORE_POSE_ZERO,-1.2,0);

	final Pose2d CONE_POSE_ONE = new Pose2d(-62, -10, Math.toRadians(180));
	final Pose2d CONE_POSE_TWO = new Pose2d(-62.5, -8.5, Math.toRadians(180));


	@Override
	public void setRobotPosition() {
		robot.drivetrain.setPose(START_POSE);
	}




	@Override
	public Command setupAuto(CommandScheduler scheduler) {

		ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.claw, robot.flip, robot.rotate, robot.extension);

		Trajectory goToPole = robot.drivetrain.getBuilder().trajectoryBuilder(START_POSE,true)
				.splineToSplineHeading(SCORE_POSE_ZERO,Math.toRadians(45))
				.build();
		Trajectory goToStack = robot.drivetrain.getBuilder().trajectoryBuilder(SCORE_POSE_ZERO,false)
				.splineToSplineHeading(CONE_POSE_ONE,Math.toRadians(180))
				.build();
		Trajectory goToStack1 = robot.drivetrain.getBuilder().trajectoryBuilder(SCORE_POSE_ONE,false)
				.splineToSplineHeading(CONE_POSE_TWO,Math.toRadians(180))
				.build();

		Trajectory fromStackToPole = robot.drivetrain.getBuilder().trajectoryBuilder(CONE_POSE_ONE,true)
				.splineToSplineHeading(SCORE_POSE_ZERO,Math.toRadians(45))
				.build();

		Trajectory fromStackToPole2 = robot.drivetrain.getBuilder().trajectoryBuilder(CONE_POSE_ONE,true)
				.splineToSplineHeading(SCORE_POSE_ONE,Math.toRadians(45))
				.build();

		return multiCommand(followRR(goToPole), commandGroups.high())
				.addNext(
						cycle(VerticalExtension.CONE_5,commandGroups,goToStack,fromStackToPole)
				)
				.addNext(
						cycle(VerticalExtension.CONE_4,commandGroups,goToStack,fromStackToPole)
				)
				.addNext(
						cycle(VerticalExtension.CONE_3,commandGroups,goToStack1,fromStackToPole2)
				)
				.addNext(
						cycle(VerticalExtension.CONE_2,commandGroups,goToStack1,fromStackToPole2)
				).addNext(
						cycle(VerticalExtension.CONE_1,commandGroups,goToStack1,fromStackToPole2)
				).addNext(commandGroups.deposit_teleop());
	}

	public Command cycle(double pickup_height, ScoringCommandGroups commandGroup, Trajectory goToStack, Trajectory fromStackToPole) {
		return multiCommand(commandGroup.deposit_generic(commandGroup.ready_for_intake_auto(pickup_height)))
				.addNext(
						followRR(goToStack)
				)
				.addNext(
						commandGroup.grab_cone_auto()
				)
				.addNext(multiCommand(followRR(fromStackToPole),commandGroup.high()));

	}

	public static Pose2d shiftRobotRelative(Pose2d poseGlobal, double robotRelativeX, double robotRelativeY) {
		// unit vector of the global Pose
		Vector2d globalVec = poseGlobal.vec();
		Vector2d globalVecUnit = poseGlobal.headingVec();//globalVec.div(globalVec.norm());  // vec / mag = unit vector
		// orthogonal vec
		Vector2d orthogonalVec = new Vector2d(-globalVecUnit.getY(),globalVecUnit.getX());

		Vector2d combinedVector = globalVec.plus(globalVecUnit.times(robotRelativeX))
				.plus(orthogonalVec.times(robotRelativeY));
		return new Pose2d(combinedVector, poseGlobal.getHeading());
	}
}
