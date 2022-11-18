package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.RR_quickstart.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RR_quickstart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.AlignWithVision2Auto;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.UltimateGoalMoment.FollowPath;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.ActivateIntakeAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.DepositAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.GoToSafeHeight;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.GoToScore;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

@Autonomous
public class BlueRight extends BaseAuto {

	public final Pose2d initialPose = new Pose2d( -37.5, 63.5, Math.toRadians(-90));
	@Override
	public Command setupAuto(CommandScheduler scheduler) {

		Pose2d placeCone = new Pose2d( -35.60741466514628, 11, Math.toRadians(308.06138282915236));
		Pose2d goNearScoring1 = new Pose2d( placeCone.getX(), 24, Math.toRadians(0));

		Pose2d placeCone2 = new Pose2d( placeCone.getX(), placeCone.getY(), placeCone.getHeading());

		Pose2d pickupFull = new Pose2d(-62,14.5,Math.toRadians(0));
		Pose2d pickupPartial = new Pose2d(-48, pickupFull.getY(),Math.toRadians(0));

		Pose2d park = new Pose2d(-12,12,Math.toRadians(0));
		TrajectoryVelocityConstraint slowConstraint = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 1.5, Math.toRadians(80),DriveConstants.TRACK_WIDTH);
		TrajectoryAccelerationConstraint slowConstraintAccel = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 1.5);


		Trajectory goToConePlacingFirst = robot.drivetrain.getBuilder().trajectoryBuilder(initialPose)
				.splineTo(goNearScoring1.vec(), Math.toRadians(270))
				.splineToSplineHeading(placeCone,Math.toRadians(270),slowConstraint,slowConstraintAccel)
				.build();

		Trajectory pickupConeFIRST = robot.drivetrain.getBuilder().trajectoryBuilder(placeCone,true)
				.splineTo(pickupPartial.vec(),Math.toRadians(180),slowConstraint,slowConstraintAccel)
				.splineToConstantHeading(pickupFull.vec(),Math.toRadians(180),slowConstraint,slowConstraintAccel)
				.build();
		Trajectory pickupCone = robot.drivetrain.getBuilder().trajectoryBuilder(placeCone2,true)
				.splineTo(pickupPartial.vec(),Math.toRadians(180),slowConstraint,slowConstraintAccel)
				.splineToConstantHeading(pickupFull.vec(),Math.toRadians(180),slowConstraint,slowConstraintAccel)
				.build();

		Trajectory placeConeTrajectory = robot.drivetrain.getBuilder().trajectoryBuilder(pickupFull)
				.splineTo(pickupPartial.vec(),Math.toRadians(0),slowConstraint,slowConstraintAccel)
				.splineTo(placeCone2.vec(), placeCone2.getHeading(),slowConstraint,slowConstraintAccel)
				.build();

		Trajectory goToPark = robot.drivetrain.getBuilder().trajectoryBuilder(placeConeTrajectory.end())
				.lineToConstantHeading(park.vec())
				.build();


		double depositDelayS = 0.6;



		return new MultipleCommand(new GoToSafeHeight(robot.scoringMechanism),followRR(goToConePlacingFirst))
				.addNext(new GoToScore(robot.scoringMechanism, ScoringMechanism.States.HIGH))
				.addNext(new DepositAuto(robot.scoringMechanism))
				.addNext(new Delay(depositDelayS))
				// go pickup and place second cone (first of stack)
				.addNext(followRR(pickupConeFIRST))
				.addNext(new ActivateIntakeAuto(robot.scoringMechanism))
				.addNext(followRR(placeConeTrajectory))
				.addNext(new GoToScore(robot.scoringMechanism, ScoringMechanism.States.HIGH))
				.addNext(new DepositAuto(robot.scoringMechanism))
				.addNext(new Delay(depositDelayS))
				// go pickup and place third cone
				.addNext(followRR(pickupCone))
				.addNext(new ActivateIntakeAuto(robot.scoringMechanism))
				.addNext(followRR(placeConeTrajectory))
				.addNext(new GoToScore(robot.scoringMechanism, ScoringMechanism.States.HIGH))
				.addNext(new DepositAuto(robot.scoringMechanism))
				.addNext(new Delay(depositDelayS))
				.addNext(followRR(goToPark));



	}

	@Override
	public void setRobotPosition() {
		robot.drivetrain.setPose(initialPose);
	}
}
