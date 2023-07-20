package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseTeleop;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RobotRelative;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.RunCommandLegacy;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.VisionUtils.VisionMode;


@TeleOp
public class Teleop extends BaseTeleop {


	@Override
	public Command setupTeleop(CommandScheduler scheduler) {
		ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.claw,robot.flip,robot.rotate,robot.extension);
		robot.gamepad1.whenTrianglePressed(commandGroups.high());
		robot.gamepad1.whenCirclePressed(commandGroups.mid());
		robot.gamepad1.whenCrossPressed(commandGroups.low());
		robot.gamepad1.whenSquarePressed(commandGroups.ground());
		robot.gamepad1.whenRightTriggerPressed(commandGroups.deposit_teleop());
		robot.gamepad1.whenRightBumperPressed(new RunCommandLegacy(commandGroups::grab_cone));
		robot.coneSensors.setCommand(new RunCommandLegacy(commandGroups::sensor_cone_grab));

		robot.gamepad1.whenDPadUpPressed(commandGroups.stack5(robot.extension.getSlideTargetPosition()));
		robot.gamepad1.whenDPadRightPressed(commandGroups.stack4());
		robot.gamepad1.whenDPadDownPressed(commandGroups.stack3());
		robot.gamepad1.whenDPadLeftPressed(commandGroups.stack2());
		return new MultipleCommand(new RobotRelative(robot, robot.gamepad1));
	}
	@Override
	public VisionMode getVisionMode() { return VisionMode.LION; }
}
