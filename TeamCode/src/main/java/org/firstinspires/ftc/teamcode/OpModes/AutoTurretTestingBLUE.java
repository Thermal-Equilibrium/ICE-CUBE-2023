package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands.ConeFollow;
import org.firstinspires.ftc.teamcode.Utils.Team;

@Disabled

@Autonomous
public class AutoTurretTestingBLUE extends BaseAuto {
	@Override
	public Command setupAuto(CommandScheduler scheduler) {
		waitForStart();
		return new ConeFollow(robot.scoringMechanism.turret, robot.backCamera);
	}

	@Override
	public Team getTeam() {
		return Team.BLUE;
	}
}
