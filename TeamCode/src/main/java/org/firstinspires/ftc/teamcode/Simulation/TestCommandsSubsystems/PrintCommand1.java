package org.firstinspires.ftc.teamcode.Simulation.TestCommandsSubsystems;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;

public class PrintCommand1 extends Command {
    protected String message;
    PrintSubsystem1 printSubsystem;
    boolean isComplete = false;

    public PrintCommand1(PrintSubsystem1 printSub, String printMessage) {
        super(printSub);

        printSubsystem = printSub;
        message = printMessage;
    }

    public void init() {
        printSubsystem.print("start: " + message);
    }

    public void periodic() {
        printSubsystem.print("periodic: " + message);
        isComplete = true;

    }

    public boolean completed() {
        return isComplete;
    }

    public void shutdown() {
        printSubsystem.print("shutdown: " + message);
    }
}
