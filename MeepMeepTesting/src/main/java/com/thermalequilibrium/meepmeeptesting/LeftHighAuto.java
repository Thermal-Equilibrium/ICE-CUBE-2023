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

public class LeftHighAuto {
	public static double MAX_VEL = 65;
	public static double MAX_ACCEL = 65;
	public static double MAX_ANG_VEL = Math.toRadians(150);
	public static double MAX_ANG_ACCEL = Math.toRadians(120);

	public static void main(String[] args) {
		MeepMeep meepMeep = new MeepMeep(800);
		final Pose2d START_POSE = new Pose2d(-31, -64.5, Math.toRadians(-90));
		final Pose2d SCORE_POSE_ZERO = shiftRobotRelative(new Pose2d(-25, -3, Math.toRadians(180 + 45)),-1,0);
		final Pose2d SCORE_POSE_ONE = shiftRobotRelative(SCORE_POSE_ZERO,-1.2,0);

		final Pose2d CONE_POSE_ONE = new Pose2d(-61, -10, Math.toRadians(180));
		final Pose2d CONE_POSE_TWO = new Pose2d(-61, -8.5, Math.toRadians(180));
		final Pose2d ZONE_ONE = new Pose2d(-52, -16, Math.toRadians(0));
		final Pose2d ZONE_TWO = new Pose2d(-32, -16, Math.toRadians(90));
		final Pose2d ZONE_THREE = new Pose2d(-8, -16, Math.toRadians(-90));
		final Pose2d go_to_score_initial = new Pose2d(-31,-40, Math.toRadians(-90));

		RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, 12)
				.followTrajectorySequence(drive ->
						drive.trajectorySequenceBuilder(START_POSE)
								.setReversed(true)
								.splineToSplineHeading(go_to_score_initial,calculateTangent(START_POSE,go_to_score_initial))
								.splineToSplineHeading(SCORE_POSE_ZERO,calculateTangent(go_to_score_initial,SCORE_POSE_ZERO))
								.setReversed(false)
								.splineToSplineHeading(CONE_POSE_ONE,Math.toRadians(180))
								.setReversed(true)
								.splineToSplineHeading(SCORE_POSE_ZERO,Math.toRadians(45))
								.setReversed(false)
								.splineToSplineHeading(ZONE_THREE,Math.toRadians(Math.toRadians(90)))

								.build()

				);

		meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
				.setDarkMode(true)
				.setBackgroundAlpha(0.95f)
				.addEntity(myBot.setDimensions(13,17))

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