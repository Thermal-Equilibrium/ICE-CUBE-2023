package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.Brake;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;

public class ToggleBrake extends Command {
    Drivetrain drivetrain;
    boolean isComplete = false;

    public ToggleBrake(Drivetrain drivetrain) {
        super(drivetrain);
        this.drivetrain = drivetrain;
    }

    @Override
    public void init() {
        isComplete = true;
        drivetrain.toggleBrakeState();
    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean completed() {
        return isComplete;
    }

    @Override
    public void shutdown() {

    }

}
