package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;
import org.firstinspires.ftc.teamcode.visionPipelines.SleeveDetection;

import java.util.HashMap;

public class VanillaLattte extends BaseAuto {
    @Override
    public Command setupAuto(CommandScheduler scheduler) {

        Pose2d startPose = new Pose2d(-36, 66.5,Math.toRadians(-90));
        Pose2d exampleEnd = new Pose2d(0, 0,Math.toRadians(0));
        Vector2d goToPole = new Vector2d(-36, 12.5);
//		Pose2d goToPole1 = new Pose2d(-35, 8, Math.toRadians(-90));
        Vector2d rotateFaceMedium = new Vector2d(-32.5, 14.5);
//		Vector2d rotateFaceMedium = new Vector2d(-32.5, 14.5);
//		Pose2d rotateFaceMedium1 = new Pose2d(-36, 6, Math.toRadians(50));
        Pose2d goToPark1 = new Pose2d(-12.5, 12.5, Math.toRadians(180));
//		Vector2d goToPark1 = new Vector2d(-12.5, 12.5);
        Pose2d goToPark3 = new Pose2d(-58, 12.5, Math.toRadians(45));
        Pose2d goToPark2 = new Pose2d(-35, 12.5, Math.toRadians(0));
//		Vector2d goToPark3 = new Vector2d(-58, 9);
//		Vector2d goToPark2 = new Vector2d(-35, 12.5);

        HashMap<SleeveDetection.ParkingPosition, Pose2d> parking = new HashMap<SleeveDetection.ParkingPosition, Pose2d>();
//		HashMap<Integer, Pose2d> toScoring = new HashMap<Integer, Pose2d>();
//		HashMap<Integer, Pose2d> rotation = new HashMap<Integer, Pose2d>();

        parking.put(SleeveDetection.ParkingPosition.LEFT, goToPark1);
        parking.put(SleeveDetection.ParkingPosition.CENTER, goToPark3);
        parking.put(SleeveDetection.ParkingPosition.RIGHT, goToPark2);

        Trajectory scoring = robot.drivetrain.getBuilder().trajectoryBuilder(startPose,false)
                .splineToConstantHeading(goToPole,Math.toRadians(270))
                .splineTo(rotateFaceMedium, Math.toRadians(45))
                .lineToLinearHeading(parking.get(parkingPosition))
                .build();

        Trajectory parkTraj = robot.drivetrain.getBuilder().trajectoryBuilder(scoring.end(),false)
                .splineToLinearHeading(parking.get(parkingPosition),Math.toRadians(0))
                .build();

        Command auto = followRR(scoring).addNext(followRR(parkTraj));

        return auto;
    }

    public void addCycle(Command command, ScoringCommandGroups commandGroups) {
        command.addNext(multiCommand(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION),
                        commandGroups.moveToIntakingRightAuto(),
                        commandGroups.moveHorizontalExtension(HorizontalExtension.PRE_EMPTIVE_EXTEND)))
                .addNext(commandGroups.moveHorizontalExtension(HorizontalExtension.mostlyAutoExtension))
                .addNext(commandGroups.collectConeAuto());
    }
}
