package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;

public class CancelableMoveArmDirect extends Command {
    public static boolean cancelled = false;
    double delayS = 0.15;

    Turret turret;
    double position;

    ElapsedTime timer = new ElapsedTime();

    public CancelableMoveArmDirect(Turret turret, double position) {
        this.turret = turret;
        this.position = position;
    }

    @Override
    public void init() {
        if (!cancelled) {
            turret.setArmDirect(position);
            timer.reset();
        }
    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean completed() {
        return (timer.seconds() > delayS) || cancelled;
    }

    @Override
    public void shutdown() {

    }
}
