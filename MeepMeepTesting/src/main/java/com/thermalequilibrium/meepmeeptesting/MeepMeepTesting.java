package com.thermalequilibrium.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
	public static void main(String[] args) {
		MeepMeep meepMeep = new MeepMeep(800);
		 final Pose2d initialPose = new Pose2d( -37.5, 63.5, Math.toRadians(-90));
		Pose2d placeCone = new Pose2d( -36.60741466514628, 11, Math.toRadians(308.06138282915236));
		Pose2d placeCone2 = new Pose2d( placeCone.getX() + 2, placeCone.getY() + 0.5, placeCone.getHeading());

		Pose2d goNearScoring1 = new Pose2d(-36.0, 18, Math.toRadians(-90));

		Pose2d goNearScoring = new Pose2d(-36.0, 18, placeCone.getHeading());

		Pose2d pickupPartial = new Pose2d(-48,18,Math.toRadians(0));
		Pose2d pickupFull = new Pose2d(-62,12,Math.toRadians(0));

		RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(60, 60, Math.toRadians(160), Math.toRadians(180), 12)
				.followTrajectorySequence(drive ->
						drive.trajectorySequenceBuilder(initialPose)
								.lineToLinearHeading(goNearScoring1)
								.lineToLinearHeading(placeCone)
								.setReversed(true)
								.splineToLinearHeading(pickupFull,Math.toRadians(180))
								.setReversed(false)
								.splineToLinearHeading(placeCone2, 0)
								.build()

				);

		meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
				.setDarkMode(true)
				.setBackgroundAlpha(0.95f)
				.addEntity(myBot.setDimensions(14.5,17))

				.start();
	}
}