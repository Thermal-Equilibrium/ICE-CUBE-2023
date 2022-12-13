package com.thermalequilibrium.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
		final Pose2d initialPose = new Pose2d( -35.5, 63.5, Math.toRadians(-90));
		Pose2d placeCone = new Pose2d(-32, 10.5, Math.toRadians(308.06138282915236));
		placeCone = shiftRobotRelative(placeCone, 2.5,-0.5);
		Pose2d goNearScoring1 = new Pose2d( -32, 24, Math.toRadians(0));

		//Pose2d placeCone2 = new Pose2d(-29.60741466514628, 10, placeCone.getHeading());
		Pose2d placeCone2 = new Pose2d(-32, 5, Math.toRadians(330));
		Pose2d pickupFull = new Pose2d(-61.5,12,Math.toRadians(0));
		Pose2d pickupPartial = new Pose2d(-48, pickupFull.getY(),Math.toRadians(0));

		Pose2d park_safe = new Pose2d(-36.60741466514628, 18, Math.toRadians(0));

		Pose2d parkLeft = new Pose2d(-9,14,Math.toRadians(0));
		Pose2d parkMid = new Pose2d(-33,14,Math.toRadians(0));
		Pose2d parkRight = new Pose2d(-57,10,Math.toRadians(0));

		Pose2d park = parkRight;

		Pose2d polePosition = new Pose2d(-24,0,0);

		double angleToPole = -Math.atan2(placeCone2.getY()-polePosition.getY(), placeCone2.getX() - polePosition.getX());

		TrajectoryVelocityConstraint slowConstraint = SampleMecanumDrive.getVelocityConstraint(40,Math.toRadians(120),12);
		TrajectoryAccelerationConstraint slowConstraintAccel = SampleMecanumDrive.getAccelerationConstraint(30);
		Pose2d finalPlaceCone = placeCone;
		Pose2d finalPlaceCone1 = placeCone;
		RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, 12)
				.followTrajectorySequence(drive ->
						drive.trajectorySequenceBuilder(initialPose)
								.splineTo(goNearScoring1.vec(), Math.toRadians(270))
								.splineToSplineHeading(finalPlaceCone1,Math.toRadians(270),slowConstraint,slowConstraintAccel)
//								.lineToLinearHeading(goNearScoring1)
//								.lineToLinearHeading(placeCone)
								.setReversed(true)
								.splineTo(pickupPartial.vec(),Math.toRadians(180))
								.splineToSplineHeading(pickupFull, Math.PI)
								.setReversed(false)
								.splineTo(pickupPartial.vec(),calculateTangent(pickupFull,pickupPartial),slowConstraint,slowConstraintAccel)
								.splineToSplineHeading(placeCone2, calculateTangent(pickupPartial,placeCone2),slowConstraint,slowConstraintAccel)
								.build()

				);

		meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
				.setDarkMode(true)
				.setBackgroundAlpha(0.95f)
				.addEntity(myBot.setDimensions(14.5,17))

				.start();
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