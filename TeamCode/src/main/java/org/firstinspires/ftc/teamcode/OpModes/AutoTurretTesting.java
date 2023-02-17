package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveTurretDirect;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.SetHorizontalExtensionInches;
import org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands.ConeFollow;
import org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands.VisualIntake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.Utils.Team;
@Disabled

@Autonomous

public class AutoTurretTesting extends BaseAuto {
	@Override
	public Command setupAuto(CommandScheduler scheduler) {
		ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain, robot.backCamera);
		waitForStart();

		return commandGroups.openClaw()
				.addNext(new VisualIntake(robot.scoringMechanism.turret, robot.backCamera,robot.scoringMechanism.horizontalExtension))
				.addNext(commandGroups.moveArm(Turret.ArmStates.DOWN))
				.addNext(new Delay(.20))
				.addNext(commandGroups.grabCone())
				.addNext(commandGroups.openLatch())
				.addNext(new Delay(.25))
				.addNext(commandGroups.collectCone())
				.addNext(commandGroups.closeLatch());
	}

	@Override
	public Team getTeam() {
		return Team.RED;
	}
}
