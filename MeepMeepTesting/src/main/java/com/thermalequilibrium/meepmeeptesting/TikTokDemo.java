package com.thermalequilibrium.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class TikTokDemo {
    public static double MAX_VEL = 40;
    public static double MAX_ACCEL = 30;
    public static double MAX_ANG_VEL = Math.toRadians(70);
    public static double MAX_ANG_ACCEL = Math.toRadians(50);


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-52,0))
                                .lineToSplineHeading(new Pose2d(-26, 0, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(0, 0, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(26, 0, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(52, 0, Math.toRadians(0)))
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot.setDimensions(14.5,17))

                .start();
    }
}