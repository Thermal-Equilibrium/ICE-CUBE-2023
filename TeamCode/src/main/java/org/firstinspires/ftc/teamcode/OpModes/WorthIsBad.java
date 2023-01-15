package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands.TurretToBestCone;

@Autonomous
public class WorthIsBad extends BaseAuto {


    @Override
    public Command setupAuto(CommandScheduler scheduler) {

        waitForStart();
        return new TurretToBestCone(robot.scoringMechanism.turret, robot.vision, false, true, true, false, false);
    }


}
