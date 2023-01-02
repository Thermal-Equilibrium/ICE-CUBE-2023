package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseTeleop;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.AlignWithDistanceSensor;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RobotRelative;

import java.util.function.BooleanSupplier;


@TeleOp
public class TestTeleop extends BaseTeleop {



	@Override
	public Command setupTeleop(CommandScheduler scheduler) {

		BooleanSupplier autoAlignSupplier = () -> gamepad1.left_stick_button;
		robot.gamepad1.whenLeftStickButtonPressed(new AlignWithDistanceSensor(robot.drivetrain,
				robot.distanceSensor,
				autoAlignSupplier).addNext(new RobotRelative(robot,robot.gamepad1)));

		//return new ClosedLoopTeleop(robot.drivetrain,robot.odometry,robot.gamepad1);
		return new RobotRelative(robot, robot.gamepad1);
	}
}
