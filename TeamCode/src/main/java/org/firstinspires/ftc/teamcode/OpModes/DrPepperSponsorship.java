package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Utils.Team;
@Disabled

@Autonomous
public class DrPepperSponsorship extends BaseAuto { // it WILL happen
	@Override
	public Command setupAuto(CommandScheduler scheduler) {
		ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain, robot.backCamera);
		waitForStart();
		return commandGroups.autoGoToCone().addNext(new Delay(.08)).addNext(commandGroups.grabCone()).addNext(commandGroups.openLatch()).addNext(new Delay(.08)).addNext(commandGroups.collectCone()).addNext(commandGroups.closeLatch());
	}

	@Override
	public Team getTeam() {
		return Team.RED;
	}
}
