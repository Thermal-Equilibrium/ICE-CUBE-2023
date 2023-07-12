package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseTeleop;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RobotRelative;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.VisionUtils.VisionMode;


@TeleOp
public class Teleop extends BaseTeleop {

	@Override
	public Command setupTeleop(CommandScheduler scheduler) {
		return new MultipleCommand(new RobotRelative(robot, robot.gamepad1));
	}
	@Override
	public VisionMode getVisionMode() { return VisionMode.LION; }
}
