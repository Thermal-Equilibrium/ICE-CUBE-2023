package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.NullCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;
import org.firstinspires.ftc.teamcode.visionPipelines.SleeveDetection;

@Autonomous
public class LeftHighPole extends BaseAuto {

	final Pose2d START_POSE = new Pose2d(-31, -64.5, Math.toRadians(-90));

	final Pose2d go_to_score_initial = new Pose2d(-31,-40, Math.toRadians(-90));
	final Pose2d SCORE_POSE_ZERO = shiftRobotRelative(new Pose2d(-21, -7, Math.toRadians(180 + 45)),-1,-2);
	final Pose2d SCORE_POSE_ONE = shiftRobotRelative(SCORE_POSE_ZERO,-1.2,2);

	final Pose2d CONE_POSE_ONE = new Pose2d(-60, -12, Math.toRadians(180));
	final Pose2d CONE_POSE_TWO = new Pose2d(-60, -14, Math.toRadians(180));
	final Pose2d CONE_POSE_THREE = shiftRobotRelative(CONE_POSE_TWO, 0, -1);

	final Pose2d ZONE_TWO = new Pose2d(-33, -16, Math.toRadians(-90));
	final Pose2d ZONE_THREE = new Pose2d(-3.5, -11, Math.toRadians(180));


	@Override
	public void setRobotPosition() {
		robot.drivetrain.setPose(START_POSE);
	}




	@Override
	public Command setupAuto(CommandScheduler scheduler) {

		ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.claw, robot.flip, robot.rotate, robot.extension);


		Trajectory goToPole = robot.drivetrain.getBuilder().trajectoryBuilder(START_POSE,true)
				.splineToSplineHeading(go_to_score_initial,calculateTangent(START_POSE,go_to_score_initial))
				.splineToSplineHeading(SCORE_POSE_ZERO,calculateTangent(go_to_score_initial,SCORE_POSE_ZERO))
				.build();
		Trajectory goToStack = robot.drivetrain.getBuilder().trajectoryBuilder(SCORE_POSE_ZERO,false)
				.splineToSplineHeading(CONE_POSE_TWO,Math.toRadians(180))
				.build();

		Trajectory goToStackBetter = robot.drivetrain.getBuilder().trajectoryBuilder(SCORE_POSE_ONE,false)
				.splineToSplineHeading(CONE_POSE_THREE,calculateTangent(SCORE_POSE_ONE,CONE_POSE_THREE))
				.build();

		Trajectory fromStackToPole = robot.drivetrain.getBuilder().trajectoryBuilder(CONE_POSE_TWO,true)
				.splineToSplineHeading(SCORE_POSE_ZERO,calculateTangent(CONE_POSE_TWO,SCORE_POSE_ZERO))
				.build();

		Trajectory fromStackToPole2 = robot.drivetrain.getBuilder().trajectoryBuilder(CONE_POSE_THREE,true)
				.splineToSplineHeading(SCORE_POSE_ONE,Math.toRadians(45))
				.build();

		Trajectory zone1 = robot.drivetrain.getBuilder().trajectoryBuilder(SCORE_POSE_ZERO,false)
				.splineToSplineHeading(CONE_POSE_TWO,Math.toRadians(180))
				.build();

		Trajectory zone2 = robot.drivetrain.getBuilder().trajectoryBuilder(SCORE_POSE_ZERO,false)
				.lineToLinearHeading(ZONE_TWO)
				.build();

		Trajectory zone3 = robot.drivetrain.getBuilder().trajectoryBuilder(SCORE_POSE_ZERO,false)
				.splineToSplineHeading(ZONE_THREE,Math.toRadians(Math.toRadians(90)))
				.build();


		Trajectory park_trajectory = zone1;

		if (parkingPosition.equals(SleeveDetection.ParkingPosition.CENTER)) {
			park_trajectory = zone2;
		}
		if (parkingPosition.equals(SleeveDetection.ParkingPosition.RIGHT)) {
			park_trajectory = zone3;
		}



		return multiCommand(followRR(goToPole), commandGroups.high())
				.addNext(
						cycle(VerticalExtension.CONE_5,commandGroups,goToStackBetter,fromStackToPole2)
				)
				.addNext(
						cycle(VerticalExtension.CONE_4,commandGroups,goToStackBetter,fromStackToPole2)
				)
				.addNext(
						cycle(VerticalExtension.CONE_3,commandGroups,goToStackBetter,fromStackToPole2)
				)
				.addNext(
						commandGroups.deposit_teleop()
				).addNext(
						followRR(park_trajectory)
				);
	}

	public Command cycle(double pickup_height, ScoringCommandGroups commandGroup, Trajectory goToStack, Trajectory fromStackToPole) {
		return commandGroup.deposit_generic(commandGroup.ready_for_intake_auto(pickup_height))
				.addNext(
						followRR(goToStack)
				)
				.addNext(
						commandGroup.grab_cone_auto()
				)
				.addNext(multiCommand(followRR(fromStackToPole),commandGroup.high()));

	}


	public Command cycle_drive_only(double pickup_height, ScoringCommandGroups commandGroup, Trajectory goToStack, Trajectory fromStackToPole) {
		return followRR(goToStack).addNext(multiCommand(followRR(fromStackToPole)));

	}


	public Command cycle_last(double pickup_height, ScoringCommandGroups commandGroup, Trajectory goToStack, Trajectory fromStackToPole) {
		return multiCommand(commandGroup.deposit_generic(commandGroup.ready_for_intake_auto(pickup_height)))
				.addNext(
						followRR(goToStack)
				)
				.addNext(
						commandGroup.grab_cone_auto()
				)
				.addNext(multiCommand(followRR(fromStackToPole),new Delay(0.7).addNext(commandGroup.high())));

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

	public static double calculateTangent(Pose2d initialPosition, Pose2d finalPosition) {
		double xd = initialPosition.getX() - finalPosition.getX();
		double yd = initialPosition.getY() - finalPosition.getY();
		return Math.atan2(yd,xd) - Math.PI;
	}

}
