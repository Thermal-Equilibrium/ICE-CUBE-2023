package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseTeleop;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.AlignWithDistanceSensor;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RobotRelative;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringMechanismCommands.TurretControl;

import java.util.function.BooleanSupplier;


@TeleOp
public class TestTeleop extends BaseTeleop {



	@Override
	public Command setupTeleop(CommandScheduler scheduler) {
		return new MultipleCommand(new RobotRelative(robot, robot.gamepad1),
									new TurretControl(robot.scoringMechanism.turret, robot.gamepad2));
	}
}
