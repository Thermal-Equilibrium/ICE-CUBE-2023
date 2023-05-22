package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.VerticalTestingCommand;

@Autonomous
public class VerticalTesting extends BaseAuto {


    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        return new VerticalTestingCommand(robot.scoringMechanism.verticalExtension);
    }
}
