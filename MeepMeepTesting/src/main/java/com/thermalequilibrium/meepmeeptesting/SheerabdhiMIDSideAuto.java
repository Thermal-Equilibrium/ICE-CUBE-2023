package com.thermalequilibrium.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.HashMap;

public class SheerabdhiMIDSideAuto {
	public static double MAX_VEL = 65;
	public static double MAX_ACCEL = 65;
	public static double MAX_ANG_VEL = Math.toRadians(150);
	public static double MAX_ANG_ACCEL = Math.toRadians(120);

	public static void main(String[] args) {
		int parkingLocation = 2;

		MeepMeep meepMeep = new MeepMeep(800);
		Pose2d startPose = new Pose2d(-36, 66.5,Math.toRadians(-90));
		Pose2d exampleEnd = new Pose2d(0, 0,Math.toRadians(0));
		Pose2d goToPoleHigh = new Pose2d(-35, 14, Math.toRadians(-70));
		Pose2d rotateFaceRight = new Pose2d(-36, 12.5, Math.toRadians(0));
		Pose2d goToPark1 = new Pose2d(-12.5, 12.5, Math.toRadians(0));
		Pose2d goToPark3 = new Pose2d(-58, 12.5, Math.toRadians(0));
		Pose2d goToPark2 = new Pose2d(-35, 12.5, Math.toRadians(0));

		HashMap<Integer, Pose2d> parking = new HashMap<Integer, Pose2d>();

		parking.put(1, goToPark1);
		parking.put(3, goToPark3);
		parking.put(2, goToPark2);

		RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, 12)
				.followTrajectorySequence(drive ->
						drive.trajectorySequenceBuilder(startPose)
								.splineToSplineHeading(goToPoleHigh,Math.toRadians(270))
								.splineToLinearHeading(rotateFaceRight,Math.toRadians(0))
								.splineToSplineHeading(parking.get(parkingLocation),Math.toRadians(0))
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
