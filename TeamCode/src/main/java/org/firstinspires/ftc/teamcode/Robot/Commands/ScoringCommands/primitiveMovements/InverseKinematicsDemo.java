package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;

public class InverseKinematicsDemo extends Command {
    public static double TIME_PER_SIDE = 3.0 * 1000; // milliseconds

    public static double SIDE_LENGTH = 10.0; // inches


    HorizontalExtension extension;
    Turret turret;
    ElapsedTime timer = new ElapsedTime();
    double x = 20;
    double y = -SIDE_LENGTH / 2;
    double z = 0;


    public InverseKinematicsDemo(HorizontalExtension extension, Turret turret) {
        this.extension = extension;
        this.turret = turret;
    }

    @Override
    public void init() {
        timer.reset();

    }

    @Override
    public void periodic() {
        if (timer.milliseconds() < TIME_PER_SIDE) {
            y = -SIDE_LENGTH / 2 + (timer.milliseconds() / TIME_PER_SIDE) * SIDE_LENGTH;
            z = 0;
        } else if (timer.milliseconds() < 2 * TIME_PER_SIDE) {
            y = SIDE_LENGTH / 2;
            z = (timer.milliseconds() - TIME_PER_SIDE) / TIME_PER_SIDE * SIDE_LENGTH;
        } else if (timer.milliseconds() < 3 * TIME_PER_SIDE) {
            y = SIDE_LENGTH / 2 - (timer.milliseconds() - 2 * TIME_PER_SIDE) / TIME_PER_SIDE * SIDE_LENGTH;
            z = SIDE_LENGTH;
        } else if (timer.milliseconds() < 4 * TIME_PER_SIDE) {
            y = -SIDE_LENGTH / 2;
            z = SIDE_LENGTH - (timer.milliseconds() - 3 * TIME_PER_SIDE) / TIME_PER_SIDE * SIDE_LENGTH;
        } else {
            timer.reset();
        }
//        turret.setArmKinematicPosition(y, z);
//        extension.setExtensionKinematicPosition(x, y, z);

    }

    @Override
    public boolean completed() {
        return false;
    }

    @Override
    public void shutdown() {

    }
}
