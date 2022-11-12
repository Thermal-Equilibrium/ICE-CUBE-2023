package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.AlignWithVision2Auto;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.ActivateIntakeAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.DepositAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.GoToScore;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

@Autonomous
public class BlueRight extends BaseAuto {

	public final Pose2d initialPose = new Pose2d( -37.5, 63.5, Math.toRadians(-90));
	@Override
	public Command setupAuto(CommandScheduler scheduler) {

		Pose2d placeCone = new Pose2d( -36.10741466514628, 12.56727416105955, Math.toRadians(308.06138282915236));
		Pose2d placeCone2 = new Pose2d( placeCone.getX() + 2, placeCone.getY() + 1, placeCone.getHeading());

		Pose2d goNearScoring1 = new Pose2d(-36.0, 18, Math.toRadians(-90));

		Pose2d goNearScoring = new Pose2d(-36.0, 18, placeCone.getHeading());

		Pose2d pickupPartial = new Pose2d(-48,18,Math.toRadians(0));
		Pose2d pickupFull = new Pose2d(-64,14.5,Math.toRadians(0));

		return goToLQR(goNearScoring1)
				.addNext(goToLQR(goNearScoring))
				.addNext(new MultipleCommand(new GoToScore(robot.scoringMechanism, ScoringMechanism.States.HIGH), goToLQR(placeCone)))
				.addNext(new AlignWithVision2Auto(robot.drivetrain, robot.distanceSensor))
				.addNext(new DepositAuto(robot.scoringMechanism))
				.addNext(new Delay(2))
				.addNext(goToLQR(pickupPartial))
				.addNext(goToLQR(pickupFull))
				.addNext(new ActivateIntakeAuto(robot.scoringMechanism))
				.addNext(goToLQR(goNearScoring))
				.addNext(new MultipleCommand(new GoToScore(robot.scoringMechanism, ScoringMechanism.States.HIGH), goToLQR(placeCone2)))
				.addNext(new AlignWithVision2Auto(robot.drivetrain, robot.distanceSensor))
				.addNext(new DepositAuto(robot.scoringMechanism))
				.addNext(new Delay(2));


	}

	@Override
	public void setRobotPosition() {
		robot.drivetrain.setPose(initialPose);
	}
}
