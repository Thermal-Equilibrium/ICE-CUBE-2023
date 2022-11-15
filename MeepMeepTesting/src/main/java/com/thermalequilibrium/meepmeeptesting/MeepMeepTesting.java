package com.thermalequilibrium.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class MeepMeepTesting {
	public static double MAX_VEL = 40;
	public static double MAX_ACCEL = 30;
	public static double MAX_ANG_VEL = Math.toRadians(70);
	public static double MAX_ANG_ACCEL = Math.toRadians(50);

	public static void main(String[] args) {
		MeepMeep meepMeep = new MeepMeep(800);
		 final Pose2d initialPose = new Pose2d( -37.5, 63.5, Math.toRadians(-90));
		Pose2d placeCone = new Pose2d( -36.60741466514628, 11, Math.toRadians(308.06138282915236));
		Pose2d placeCone2 = new Pose2d( placeCone.getX() + 2, placeCone.getY() + 0.5, placeCone.getHeading());

		Pose2d goNearScoring1 = new Pose2d(-36.0, 18, Math.toRadians(-90));

		Pose2d goNearScoring = new Pose2d(-36.0, 18, placeCone.getHeading());

		Pose2d pickupPartial = new Pose2d(-48,12,Math.toRadians(0));
		Pose2d pickupFull = new Pose2d(-62,12,Math.toRadians(0));

		TrajectoryVelocityConstraint slowConstraint = SampleMecanumDrive.getVelocityConstraint(30,Math.toRadians(120),12);
		TrajectoryAccelerationConstraint slowConstraintAccel = SampleMecanumDrive.getAccelerationConstraint(25);
		RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, 12)
				.followTrajectorySequence(drive ->
						drive.trajectorySequenceBuilder(initialPose)
								.lineToLinearHeading(goNearScoring1)
								.lineToLinearHeading(placeCone)
								.setReversed(true)
								.splineTo(pickupPartial.vec(),Math.toRadians(180))
								.splineToConstantHeading(pickupFull.vec(),Math.toRadians(180))
								.setReversed(false)
								.splineTo(pickupPartial.vec(),Math.toRadians(0))
								.splineTo(placeCone2.vec(), placeCone2.getHeading(),slowConstraint,slowConstraintAccel)
								.build()

				);

		meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
				.setDarkMode(true)
				.setBackgroundAlpha(0.95f)
				.addEntity(myBot.setDimensions(14.5,17))

				.start();
	}
}