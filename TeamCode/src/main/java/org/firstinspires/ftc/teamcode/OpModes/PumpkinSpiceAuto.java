package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.RR_quickstart.util.BasedMath.shiftRobotRelative;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.Brake.ToggleBrake;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RoadrunnerHoldPose;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.NullCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.RunCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.SafetyAlgorithms.FollowIfNeeded;
import org.firstinspires.ftc.teamcode.Robot.Commands.SafetyAlgorithms.MoveHorizontalWhenNeeded;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveHorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveHorizontalExtensionAsync;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;
import org.firstinspires.ftc.teamcode.Utils.Team;

import java.util.function.BooleanSupplier;

@Autonomous
public class PumpkinSpiceAuto extends BaseAuto {


	final Pose2d goToPole1 = new Pose2d(-38, 24, Math.toRadians(-100));
	final Pose2d parkRight = new Pose2d(-63, 20, Math.toRadians(0));
	final Pose2d parkMID = new Pose2d(-40, 18, Math.toRadians(-90));
	//    final Pose2d parkLeft1 = new Pose2d(-38,26,Math.toRadians(-90));
//    final Pose2d parkLeft = new Pose2d(-6,38,Math.toRadians(180));
	final Pose2d parkLeft1_new = new Pose2d(-38, 19, Math.toRadians(270));
	final Pose2d parkLeft_new = new Pose2d(-8, 15, Math.toRadians(90));
	Pose2d startPose = new Pose2d(-36, 66.5, Math.toRadians(-90));
	Pose2d goToPole2 = shiftRobotRelative(
			new Pose2d(-36.2, 10.158013549498268, Math.toRadians(338.11832672430523)),
			0.6,
			-1
	);
	final Pose2d parkRight1 = new Pose2d(goToPole2.getX() - 1, goToPole2.getY() + 3, goToPole2.getHeading());
	Pose2d DislodgePosition = shiftRobotRelative(goToPole2, -2,12);

	@Override
	public Team getTeam() {
		return Team.BLUE;
	}

	@Override
	public void setRobotPosition() {
		robot.drivetrain.setPose(startPose);
	}

	@Override
	public Command setupAuto(CommandScheduler scheduler) {
		ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain, robot.backCamera);

//		 scheduler.addIntent(new ToggleBrake(robot.drivetrain).interruptOthers().when(() -> robot.drivetrain.getVelocity().getX() == 0));

		Trajectory driveToPole = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
				.splineTo(goToPole1.vec(), goToPole1.getHeading())
				.splineToSplineHeading(goToPole2, calculateTangent(goToPole1, goToPole2))
				.build();

		Trajectory parkRightTraj = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2, true)
				.splineToConstantHeading(parkRight1.vec(), calculateTangent(goToPole2, parkRight1))
				.splineToSplineHeading(parkRight, calculateTangent(parkRight1, parkRight))
				.build();

		Trajectory parkMidTraj = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2, true)
				.lineToLinearHeading(parkMID)
				.build();
//
//        Trajectory parkLeftTraj = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2,true)
//                .splineTo(parkLeft1.vec(),Math.toRadians(75))
//                .splineTo(parkLeft.vec(), Math.toRadians(0))
//                .build();

		Trajectory parkLeftTrajNew = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2, true)
				.splineToConstantHeading(parkLeft1_new.vec(), Math.toRadians(0))
				.splineToSplineHeading(parkLeft_new, Math.toRadians(0))
				.build();




		Trajectory park = parkLeftTrajNew;

		switch (parkingPosition) {
			case LEFT:
				park = parkLeftTrajNew;
				break;
			case CENTER:
				park = parkMidTraj;
				break;
			case RIGHT:
				park = parkRightTraj;
				break;
		}

		Command auto = followRR(driveToPole);


		auto.addNext(new RoadrunnerHoldPose(robot, goToPole2));
		for (int i = 0; i < 5; ++i) {
			addCycle(auto, commandGroups);
		}

		auto.addNext(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION))
				.addNext(new Delay(0.1))
				.addNext(commandGroups.depositCone());

		auto.addNext(commandGroups.moveHorizontalExtension(0));
		auto.addNext(new Delay(0.1));
		auto.addNext(followRR(park));
		return auto;
	}

	public void addCycle(Command command, ScoringCommandGroups commandGroups) {


		Command nextCommand = multiCommand(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION),
				commandGroups.moveToIntakingRightAuto(),
				commandGroups.moveHorizontalExtension(HorizontalExtension.PRE_EMPTIVE_EXTEND))
				.addNext(commandGroups.moveHorizontalExtension(HorizontalExtension.mostlyAutoExtension))
				.addNext(commandGroups.collectConeAutoPT1(HorizontalExtension.autoExtension))
				.addNext(DislodgeCone())
				.addNext(commandGroups.collectConeAutoPT2());

		command.addNext(nextCommand);

	}


	public Command DislodgeCone() {

		Trajectory DislodgeCone1 = robot.drivetrain.getBuilder().trajectoryBuilder(goToPole2,false)
				.lineToConstantHeading(DislodgePosition.vec())
				.build();

		Trajectory DislodgeCone2 = robot.drivetrain.getBuilder().trajectoryBuilder(DislodgePosition,false)
				.lineToConstantHeading(goToPole2.vec())
				.build();


		return followIfNeeded(robot,DislodgeCone1, () -> !robot.scoringMechanism.horizontalExtension.currentExceedsCutoff())
					.addNext(new MoveHorizontalWhenNeeded(robot.scoringMechanism.horizontalExtension, HorizontalExtension.DISLODGE_CONE, () -> !robot.scoringMechanism.horizontalExtension.currentExceedsCutoff()))
					.addNext(followIfNeeded(robot,DislodgeCone2, () -> !robot.scoringMechanism.horizontalExtension.currentExceedsCutoff()));


	}

}
