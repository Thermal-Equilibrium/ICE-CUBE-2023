package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseTeleop;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.Break.ToggleBreak;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RobotRelative;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringMechanismCommands.TurretControl;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;


@TeleOp
public class Teleop extends BaseTeleop {


	@Override
	public Command setupTeleop(CommandScheduler scheduler) {
		ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism);


		robot.gamepad1.whenDPadDownPressed(commandGroups.moveToIntakingLeft());
		robot.gamepad1.whenDPadLeftPressed(commandGroups.moveToIntakingLeftClosePole());
		robot.gamepad1.whenDPadRightPressed(commandGroups.moveToIntakingRightClosePole());
		robot.gamepad1.whenLeftStickButtonPressed(new ToggleBreak(robot.drivetrain));
		robot.gamepad1.whenRightBumperPressed(commandGroups.collectCone());
		robot.gamepad1.whenRightTriggerPressed(commandGroups.moveVerticalExtension(VerticalExtension.IN_POSITION));
		robot.gamepad1.whenTrianglePressed(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION));

		return new MultipleCommand(new RobotRelative(robot, robot.gamepad1),
									new TurretControl(robot.scoringMechanism.turret, robot.gamepad2));
	}
}
