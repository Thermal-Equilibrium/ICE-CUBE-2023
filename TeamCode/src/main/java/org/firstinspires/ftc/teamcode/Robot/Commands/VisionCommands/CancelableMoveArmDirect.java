package org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.VisionUtils.IntakeParameters;

public class CancelableMoveArmDirect extends Command {
    private final Turret turret;
    private final double position;

    private final ElapsedTime timer = new ElapsedTime();
    private final double delayS = 0.15;
    private final IntakeParameters intakeParameters;

    public CancelableMoveArmDirect(IntakeParameters intakeParameters, Turret turret, double position) {
        this.intakeParameters = intakeParameters;
        this.turret = turret;
        this.position = position;
    }

    @Override
    public void init() {
        if (intakeParameters.foundCone) {
            turret.setArmDirect(position);
            timer.reset();
        }
    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean completed() {
        return (timer.seconds() > delayS) || !intakeParameters.foundCone;
    }

    @Override
    public void shutdown() {

    }
}
