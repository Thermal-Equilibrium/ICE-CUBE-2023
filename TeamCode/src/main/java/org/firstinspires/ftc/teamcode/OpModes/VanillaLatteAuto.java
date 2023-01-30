package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.RR_quickstart.util.BasedMath.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.RR_quickstart.util.BasedMath;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;
import org.firstinspires.ftc.teamcode.visionPipelines.SleeveDetection;

import java.util.HashMap;
import java.util.Objects;

@Autonomous
public class VanillaLatteAuto extends BaseAuto {
    Pose2d startPose = new Pose2d(-36, 66.5,Math.toRadians(-90));

    @Override
    public void setRobotPosition() {
        robot.drivetrain.setPose(startPose);
    }

    @Override
    public Command setupAuto(CommandScheduler scheduler) {

        ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism,robot.drivetrain);

        Vector2d goToPole = new Vector2d(-36, 23);
        Pose2d rotateFaceMedium = shiftRobotRelative(new Pose2d(-34, 27,Math.toRadians(21.8816732757)),-2,0);

        Pose2d goToPark1 = new Pose2d(-12.5, 12.5, Math.toRadians(180));
        Pose2d goToPark3 = new Pose2d(-58, 12.5, Math.toRadians(45));
        Pose2d goToPark2 = new Pose2d(-35, 12.5, Math.toRadians(0));

        HashMap<SleeveDetection.ParkingPosition, Pose2d> parking = new HashMap<>();

        parking.put(SleeveDetection.ParkingPosition.LEFT, goToPark1);
        parking.put(SleeveDetection.ParkingPosition.CENTER, goToPark3);
        parking.put(SleeveDetection.ParkingPosition.RIGHT, goToPark2);

        Trajectory scoring1 = robot.drivetrain.getBuilder().trajectoryBuilder(startPose,false)
                .splineToConstantHeading(goToPole,Math.toRadians(270))
                .build();
        Trajectory scoring2 = robot.drivetrain.getBuilder().trajectoryBuilder(scoring1.end(),false)
                .lineToLinearHeading(rotateFaceMedium)
                .build();
        Trajectory parkTraj = robot.drivetrain.getBuilder().trajectoryBuilder(scoring2.end(),false)
                .lineToLinearHeading(Objects.requireNonNull(parking.get(parkingPosition)))
                .build();

        Command auto = followRR(scoring1).addNext(followRR(scoring2));
        for (int i = 0; i < 5; ++i) {
            addCycle(auto,commandGroups);
        }
        auto.addNext(commandGroups.moveVerticalExtension(VerticalExtension.MID_POSITION))
                .addNext(commandGroups.depositCone());

        auto = auto.addNext(followRR(parkTraj));
        return auto;
    }

    public void addCycle(Command command, ScoringCommandGroups commandGroups) {
        command.addNext(multiCommand(commandGroups.moveVerticalExtension(VerticalExtension.MID_POSITION),
                        commandGroups.moveToIntakingLeftAuto(),
                        commandGroups.moveHorizontalExtension(HorizontalExtension.PRE_EMPTIVE_EXTEND)))
                .addNext(commandGroups.moveHorizontalExtension(HorizontalExtension.mostlyAutoExtension_MID))
                .addNext(commandGroups.collectConeAuto(HorizontalExtension.autoExtension_MID));
    }
}
