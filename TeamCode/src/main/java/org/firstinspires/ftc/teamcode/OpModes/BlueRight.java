package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.RR_quickstart.util.BasedMath.shiftRobotRelative;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

	public final Pose2d initialPose = new Pose2d( -35.5, 63.5, Math.toRadians(-90));
	Pose2d driftVector = new Pose2d(0,0,0);

	@Override
	public Command setupAuto(CommandScheduler scheduler) {

		Pose2d placeCone = new Pose2d(-32, 10.5, Math.toRadians(308.06138282915236));
		placeCone = shiftRobotRelative(placeCone, 2.5,0.5);
		Pose2d goNearScoring1 = new Pose2d( -38, 24, Math.toRadians(0));

		//Pose2d placeCone2 = new Pose2d(-29.60741466514628, 10, placeCone.getHeading());
		Pose2d placeCone2 = new Pose2d(-32, 5, Math.toRadians(330));
		placeCone2 = shiftRobotRelative(placeCone2, 0,3);
		Pose2d pickupFull = new Pose2d(-61.2,12,Math.toRadians(0));
		Pose2d pickupPartial = new Pose2d(-48, pickupFull.getY(),Math.toRadians(0));

		Pose2d park_safe = new Pose2d(-36.60741466514628, 15, Math.toRadians(0));

		Pose2d parkLeft = new Pose2d(-9,14,Math.toRadians(0));
		Pose2d parkMid = new Pose2d(-33,14,Math.toRadians(0));
		Pose2d parkRight = new Pose2d(-57,10,Math.toRadians(0));

		Pose2d park = parkRight;
		switch (parkingPosition) {
			case LEFT:
				park = parkLeft;
				break;
			case CENTER:
				park = parkMid;
				break;
		}

		TrajectoryVelocityConstraint slowConstraint = SampleMecanumDrive.getVelocityConstraint(27, Math.toRadians(80),DriveConstants.TRACK_WIDTH);
		TrajectoryAccelerationConstraint slowConstraintAccel = SampleMecanumDrive.getAccelerationConstraint(21);


		Trajectory goToConePlacingFirst = robot.drivetrain.getBuilder().trajectoryBuilder(initialPose)
				.splineTo(goNearScoring1.vec(), Math.toRadians(270))
				.splineToSplineHeading(placeCone,calculateTangent(goNearScoring1,placeCone),slowConstraint,slowConstraintAccel)
				.build();

		Trajectory pickupConeFIRST = robot.drivetrain.getBuilder().trajectoryBuilder(placeCone,true)
				.splineTo(pickupPartial.vec(),Math.toRadians(180))
				.splineToSplineHeading(pickupFull,Math.toRadians(180))
				.build();
		Trajectory pickupCone = robot.drivetrain.getBuilder().trajectoryBuilder(placeCone2,true)
				.splineTo(pickupPartial.vec(),Math.toRadians(180))
				.splineToSplineHeading(pickupFull,Math.toRadians(180))
				.build();

		Trajectory placeConeTrajectory = robot.drivetrain.getBuilder().trajectoryBuilder(pickupFull)
				.splineTo(pickupPartial.vec(),calculateTangent(pickupFull,pickupPartial),slowConstraint,slowConstraintAccel)
				.splineToSplineHeading(placeCone2, calculateTangent(pickupPartial,placeCone2),slowConstraint,slowConstraintAccel)
				.build();

		Trajectory goToPark1 = robot.drivetrain.getBuilder().trajectoryBuilder(placeConeTrajectory.end())
				.lineToConstantHeading(park_safe.vec())
				.build();


		Trajectory goToPark2 = robot.drivetrain.getBuilder().trajectoryBuilder(goToPark1.end())
				.lineToLinearHeading(park)
				.build();


		double depositDelayS = 1;

		double depositUpDelayS = 0.4;


		return multiCommand(new GoToSafeHeight(robot.scoringMechanism),followRR(goToConePlacingFirst))
				.addNext(goToScore())
				.addNext(getPoleContextualPosition()) // yeah
				.addNext(deposit())
				.addNext(wait(depositDelayS))
				// go pickup and place second cone (first of stack)
				.addNext(followRR(pickupConeFIRST))
				.addNext(intake())
				.addNext(multiCommand(followRR(placeConeTrajectory), delayCommand(depositUpDelayS, goToScore())))
				//.addNext(relocalizeRobot())
				.addNext(deposit())

				.addNext(wait(depositDelayS))
				// go pickup and place third cone
				.addNext(followRR(pickupCone))
				.addNext(intake())
				.addNext(multiCommand(followRR(placeConeTrajectory), delayCommand(depositUpDelayS, goToScore())))
				//.addNext(relocalizeRobot())
				.addNext(deposit())

				.addNext(wait(depositDelayS))

				// go pickup and place fourth cone
				.addNext(followRR(pickupCone))
				.addNext(intake())
				.addNext(multiCommand(followRR(placeConeTrajectory), delayCommand(depositUpDelayS, goToScore())))
				//.addNext(relocalizeRobot())
				.addNext(deposit())

				.addNext(wait(depositDelayS))

				// park
				.addNext(followRR(goToPark1))
				.addNext(followRR(goToPark2));
		
	}

	@Override
	public void setRobotPosition() {
		robot.drivetrain.setPose(initialPose);
	}
}
