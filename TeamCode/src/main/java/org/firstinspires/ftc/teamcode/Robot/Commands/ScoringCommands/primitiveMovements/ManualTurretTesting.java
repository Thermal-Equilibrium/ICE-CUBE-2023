package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;

public class ManualTurretTesting extends Command {
    @Config
    public static class ManualTurretControl {
        public static double DEGREES = 180;
    }

    Turret turret;

    public ManualTurretTesting(Turret turret) {
        this.turret = turret;
    }

    @Override
    public void init() {
        turret.setBasedTurretPosition(Math.toRadians(ManualTurretControl.DEGREES));
    }

    @Override
    public void periodic() {
        turret.setBasedTurretPosition(Math.toRadians(ManualTurretControl.DEGREES));
    }

    @Override
    public boolean completed() {
        return false;
    }

    @Override
    public void shutdown() {

    }
}

