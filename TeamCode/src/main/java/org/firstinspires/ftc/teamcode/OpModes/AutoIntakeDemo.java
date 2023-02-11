package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Utils.Team;

@Autonomous
public class AutoIntakeDemo extends BaseAuto {
	@Override
	public Command setupAuto(CommandScheduler scheduler) {
		ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain, robot.backCamera);
		waitForStart();
		return commandGroups.autoIntakeCmd().addNext(new Delay(.5)).addNext(commandGroups.autoIntakeCmd()).addNext(new Delay(.5)).addNext(commandGroups.autoIntakeCmd());
	}

	@Override
	public Team getTeam() {
		return Team.RED;
	}
}
