package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;

import java.util.HashMap;

public class VanillaLattte extends BaseAuto {
    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        int parkingLocation = 1;

        Pose2d startPose = new Pose2d(-36, 66.5,Math.toRadians(-90));
        Pose2d exampleEnd = new Pose2d(0, 0,Math.toRadians(0));
        Pose2d goToPole23 = new Pose2d(-35, 14, Math.toRadians(-70));
        Pose2d goToPole1 = new Pose2d(-35, 8, Math.toRadians(-90));
        Pose2d rotateFaceMedium23 = new Pose2d(-36, 12.5, Math.toRadians(45));
        Pose2d rotateFaceMedium1 = new Pose2d(-36, 6, Math.toRadians(50));
        Pose2d goToPark1 = new Pose2d(-12.5, 12.5, Math.toRadians(0));
        Pose2d goToPark3 = new Pose2d(-58, 9, Math.toRadians(0));
        Pose2d goToPark2 = new Pose2d(-35, 12.5, Math.toRadians(45));

        HashMap<Integer, Pose2d> parking = new HashMap<Integer, Pose2d>();
        HashMap<Integer, Pose2d> toScoring = new HashMap<Integer, Pose2d>();
        HashMap<Integer, Pose2d> rotation = new HashMap<Integer, Pose2d>();

        parking.put(1, goToPark1);
        parking.put(3, goToPark3);
        parking.put(2, goToPark2);

        toScoring.put(1, goToPole1);
        toScoring.put(2, goToPole23);
        toScoring.put(3, goToPole23);

        rotation.put(1, rotateFaceMedium1);
        rotation.put(2, rotateFaceMedium23);
        rotation.put(3, rotateFaceMedium23);

        Trajectory scoring = robot.drivetrain.getBuilder().trajectoryBuilder(startPose,false)
                .splineToSplineHeading(toScoring.get(parkingLocation),Math.toRadians(270))
                .splineToLinearHeading(rotation.get(parkingLocation),Math.toRadians(0))
                .build();

        Trajectory parkTraj = robot.drivetrain.getBuilder().trajectoryBuilder(scoring.end(),false)
                .splineToLinearHeading(parking.get(parkingLocation),Math.toRadians(0))
                .build();

        Command auto = followRR(scoring).addNext(followRR(parkTraj));

        return auto;
    }
}
