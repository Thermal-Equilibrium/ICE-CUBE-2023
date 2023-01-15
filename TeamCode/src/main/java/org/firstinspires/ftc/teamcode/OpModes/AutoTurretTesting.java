package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands.TurretToBestCone;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.visionPipelines.Color;

@Autonomous
public class AutoTurretTesting extends BaseAuto {
    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        Robot.team = Color.RED;
        waitForStart();
        return new TurretToBestCone(robot.scoringMechanism.turret, robot.vision, false, true, true, false, false);
    }
}
