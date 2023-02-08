package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Utils.Team;

@Autonomous
public class AutoIntake extends BaseAuto {
    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain, robot.backCamera);
        waitForStart();
        return commandGroups.autoGoToCone().addNext(new Delay(.25)).addNext(commandGroups.grabCone()).addNext(commandGroups.openLatch()).addNext(new Delay(.25)).addNext(commandGroups.collectCone()).addNext(commandGroups.closeLatch());
//        return new NullCommand();
    }
    @Override
    public Team getTeam() {
        return Team.RED;
    }
}
