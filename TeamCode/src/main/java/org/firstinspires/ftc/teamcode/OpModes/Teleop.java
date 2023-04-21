package org.firstinspires.ftc.teamcode.OpModes;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseTeleop;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.Brake.ToggleBrake;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RobotRelative;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.NullCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.RunCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;
import org.firstinspires.ftc.teamcode.VisionUtils.VisionMode;


public class Teleop extends BaseTeleop {


	@Override
	public Command setupTeleop(CommandScheduler scheduler) {

		ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain, robot.backCamera);

		robot.gamepad1.whenDPadDownPressed(commandGroups.moveToIntakingLeft());
//		robot.gamepad1.whenDPadLeftPressed(new NullCommand());
//		robot.gamepad1.whenLeftBumperPressed(commandGroups.cornerScore());
//		robot.gamepad1.whenDPadRightPressed(new RunCommand(commandGroups::fastTeleAutoSingleMeasure));
//		robot.gamepad1.whenLeftStickButtonPressed(new ToggleBrake(robot.drivetrain));
		robot.gamepad1.whenRightBumperPressed(new RunCommand(commandGroups::collectCone));
		robot.gamepad1.whenRightTriggerPressed(new RunCommand(commandGroups::moveVerticalExtensionDownOrReleaseClaw));
		robot.gamepad1.whenSquarePressed(commandGroups.moveVerticalExtension(VerticalExtension.MID_POSITION_teleop));
//		robot.gamepad1.whenCrossPressed(new RunCommand(commandGroups::moveToIntakingLeftWithDeposit));
		robot.gamepad1.whenTrianglePressed(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION));
		robot.gamepad1.whenCirclePressed(commandGroups.moveToLowGoalScoring());
		robot.backCamera.setVisionMode(VisionMode.LION); //TODO this might be redundant, remove this if pipeline instance is created at the start of every opmode
		return new MultipleCommand(new RobotRelative(robot, robot.gamepad1));
	}
	@Override
	public VisionMode getVisionMode() { return VisionMode.LION; }
}
