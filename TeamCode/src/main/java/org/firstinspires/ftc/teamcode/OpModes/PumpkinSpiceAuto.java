package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.RR_quickstart.util.BasedMath.shiftRobotRelative;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.Break.ToggleBreak;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RoadrunnerHoldPose;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.NullCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;

@Autonomous
public class PumpkinSpiceAuto extends BaseAuto {

    Pose2d startPose = new Pose2d(-36, 66.5,Math.toRadians(-90));
    final Pose2d goToPole1 = new Pose2d(-38, 24,Math.toRadians(-100));
    Pose2d goToPole2 = shiftRobotRelative(
            new Pose2d(-34.714046022304565,10.158013549498268,Math.toRadians(338.11832672430523)),
            -2.5,
            -0.5
    );

    final Pose2d parkRight1= new Pose2d(goToPole2.getX() - 1, goToPole2.getY() + 1, goToPole2.getHeading());
    final Pose2d parkRight = new Pose2d(-63, 12, Math.toRadians(0));
    final Pose2d parkMID = new Pose2d(-40, 18, Math.toRadians(-90));
    final Pose2d parkLeft1 = new Pose2d(-36,24,Math.toRadians(-90));
    final Pose2d parkLeft = new Pose2d(-6,38,Math.toRadians(180));

    @Override
    public void setRobotPosition() {
        robot.drivetrain.setPose(startPose);
    }
    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain);

        Trajectory driveToPole = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                .splineTo(goToPole1.vec(),goToPole1.getHeading())
                .splineToSplineHeading(goToPole2,calculateTangent(goToPole1,goToPole2))
                .build();

        Trajectory parkRightTraj = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2,true)
                .splineToConstantHeading(parkRight1.vec(),calculateTangent(goToPole2,parkRight1))
                .splineToSplineHeading(parkRight,calculateTangent(parkRight1, parkRight))
                .build();

        Trajectory parkMidTraj = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2,true)
                .lineToLinearHeading(parkMID)
                .build();

        Trajectory parkLeftTraj = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2,true)
                .splineTo(parkLeft1.vec(),Math.toRadians(75))
                .splineTo(parkLeft.vec(), Math.toRadians(0))
                .build();

        Trajectory park = parkLeftTraj;

        switch (parkingPosition) {
            case LEFT:
                park = parkLeftTraj;
                break;
            case CENTER:
                park = parkMidTraj;
                break;
            case RIGHT:
                park = parkRightTraj;
                break;
        }

        Command auto = followRR(driveToPole);


         auto.addNext(new ToggleBreak(robot.drivetrain));
//        auto.addNext(new RoadrunnerHoldPose(robot,goToPole2));
        for (int i = 0; i < 5; ++i) {
            addCycle(auto,commandGroups);
        }

        auto.addNext(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION))
                .addNext(commandGroups.depositCone());

        auto.addNext(new Delay(0.1).addNext(new ToggleBreak(robot.drivetrain)));
        auto.addNext(followRR(park));
        return auto;
    }

    public void addCycle(Command command, ScoringCommandGroups commandGroups) {
        command.addNext(multiCommand(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION),
                        commandGroups.moveToIntakingRightAuto(),
                        commandGroups.moveHorizontalExtension(HorizontalExtension.PRE_EMPTIVE_EXTEND)))
                .addNext(commandGroups.moveHorizontalExtension(HorizontalExtension.autoExtension))
                .addNext(commandGroups.collectConeAuto());
    }
}
