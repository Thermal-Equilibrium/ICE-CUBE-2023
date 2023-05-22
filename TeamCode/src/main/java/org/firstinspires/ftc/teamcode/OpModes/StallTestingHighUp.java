package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Robot.Commands.SafetyAlgorithms.SafeVerticalExtension.SafeVerticalExtensionWithBackup;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.SafetyAlgorithms.SafeVerticalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;

@Autonomous
public class StallTestingHighUp extends BaseAuto {


    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        return SafeVerticalExtensionWithBackup(robot.scoringMechanism.verticalExtension, VerticalExtension.HIGH_POSITION)
//                .addNext(new SafeVerticalExtension(robot.scoringMechanism.verticalExtension, VerticalExtension.IN_POSITION))
                ;
    }
}

