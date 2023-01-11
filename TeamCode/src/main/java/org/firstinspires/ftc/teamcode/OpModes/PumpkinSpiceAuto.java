package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.RR_quickstart.util.BasedMath.shiftRobotRelative;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.Break.ToggleBreak;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.NullCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;

@Autonomous
public class PumpkinSpiceAuto extends BaseAuto {

    Pose2d startPose = new Pose2d(-36, 66.5,Math.toRadians(-90));
    final Pose2d goToPole1 = new Pose2d(-36, 24,Math.toRadians(-90));
    final Pose2d goToPole2 = new Pose2d(-35.2273297,10.137647235973153,Math.toRadians(327.48678));
    final Pose2d parkRight = new Pose2d(-60, 12, Math.toRadians(0));
    final Pose2d parkMID = new Pose2d(-40, 18, Math.toRadians(-90));
    final Pose2d parkLeft1 = new Pose2d(-36,24,Math.toRadians(-90));
    final Pose2d parkLeft = new Pose2d(-12,36,Math.toRadians(180));

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

        Trajectory parkRightTraj = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2,true)
                .splineToSplineHeading(parkRight,calculateTangent(goToPole2, parkRight))
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

        //Command auto = followRR(driveToPole);


        Command auto = new NullCommand();

        for (int i = 0; i < 5; ++i) {
            addCycle(auto,commandGroups);
        }

        auto.addNext(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION))
                .addNext(commandGroups.moveVerticalExtension(VerticalExtension.IN_POSITION));

        //auto.addNext(followRR(park));
        return auto;
    }

    public void addCycle(Command command, ScoringCommandGroups commandGroups) {
        command.addNext(multiCommand(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION),
                        commandGroups.moveHorizontalExtension(HorizontalExtension.autoExtension),
                        commandGroups.moveToIntakingRightAuto()))

                .addNext(multiCommand(commandGroups.moveVerticalExtension(VerticalExtension.IN_POSITION),
                        commandGroups.collectConeAuto()));
    }
}
