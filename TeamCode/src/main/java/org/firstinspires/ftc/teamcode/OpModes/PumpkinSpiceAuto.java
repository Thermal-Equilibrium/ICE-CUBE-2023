package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.RR_quickstart.util.BasedMath.shiftRobotRelative;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.Break.ToggleBreak;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.RunCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;

@Autonomous
public class PumpkinSpiceAuto extends BaseAuto {

    Pose2d startPose = new Pose2d(-36, 66,Math.toRadians(-90));
    final Pose2d goToPole1 = new Pose2d(-36, 24,Math.toRadians(-90));
    final Pose2d goToPole2 = shiftRobotRelative(new Pose2d(-34,6,Math.toRadians(330)),-3,1);

    @Override
    public void setRobotPosition() {
        robot.drivetrain.setPose(startPose);
    }
    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism);

        Trajectory driveToPole = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                .splineTo(goToPole1.vec(),goToPole1.getHeading())
                .splineToSplineHeading(goToPole2,calculateTangent(goToPole1,goToPole2))
                .build();

        switch (parkingPosition) {
            case LEFT:
                break;
            case CENTER:
                break;
            case RIGHT:
                break;
        }

//        return commandGroups.moveToIntakingLeft()
//                .addNext(commandGroups.collectCone())
//                .addNext(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION))
//                .addNext(commandGroups.moveVerticalExtension(VerticalExtension.IN_POSITION));

        Command cycle = new RunCommand(() -> {
            return commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION)
                    .addNext(commandGroups.moveVerticalExtension(VerticalExtension.IN_POSITION))
                    .addNext(commandGroups.moveToIntakingRightAuto())
                    .addNext(commandGroups.collectCone())
                    .addNext(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION))
                    .addNext(commandGroups.moveVerticalExtension(VerticalExtension.IN_POSITION));
        });

        return followRR(driveToPole)
                .addNext(new ToggleBreak(robot.drivetrain)) // turn break on
                .addNext(cycle)
                .addNext(cycle)
                .addNext(cycle);
    }
}
