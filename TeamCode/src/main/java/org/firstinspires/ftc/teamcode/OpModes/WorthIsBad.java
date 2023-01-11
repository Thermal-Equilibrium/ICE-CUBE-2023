package org.firstinspires.ftc.teamcode.OpModes;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands.VisionTest;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;

@Autonomous
public class WorthIsBad extends BaseAuto {


    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        waitForStart();
//        ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism);
//        Command auto = commandGroups.moveTurretDirect(1);
        Command auto = new VisionTest(robot.scoringMechanism.turret, robot.vision);
//        waitForStart();
        return auto;
    }


}
