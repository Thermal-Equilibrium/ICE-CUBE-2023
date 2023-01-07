package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;

@Autonomous
public class PumpkinSpiceAuto extends BaseAuto {
    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism);

        return commandGroups.moveToIntakingLeft()
                .addNext(commandGroups.collectCone())
                .addNext(commandGroups.moveVerticalExtension(VerticalExtension.HIGH_POSITION))
                .addNext(commandGroups.moveVerticalExtension(VerticalExtension.IN_POSITION));
    }
}
