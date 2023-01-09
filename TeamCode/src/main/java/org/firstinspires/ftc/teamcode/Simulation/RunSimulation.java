package org.firstinspires.ftc.teamcode.Simulation;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Simulation.TestCommandsSubsystems.PrintCommand1;
import org.firstinspires.ftc.teamcode.Simulation.TestCommandsSubsystems.PrintSubsystem1;

// TODO: Add actual test cases lol

public class RunSimulation {
    public static void main(String[] args) {
        PrintSubsystem1 printSub1 = new PrintSubsystem1();
        PrintSubsystem1 printSub2 = new PrintSubsystem1();
        PrintSubsystem1 printSub3 = new PrintSubsystem1();
        PrintCommand1 printCommand1 = new PrintCommand1(printSub1, "Test 1");
        PrintCommand1 printCommand12 = new PrintCommand1(printSub1, "Test 12");

        PrintCommand1 printCommand2 = new PrintCommand1(printSub2, "Test 2");
        PrintCommand1 printCommand22 = new PrintCommand1(printSub2, "Test 22");

        PrintCommand1 printCommand3 = new PrintCommand1(printSub3,"Test 3");
        PrintCommand1 printCommand33 = new PrintCommand1(printSub3, "Test 33");

        Command print1 = printCommand1.addNext(printCommand12);
        Command print2 = printCommand2.addNext(printCommand22);
        Command print3 = new MultipleCommand(printCommand3,printCommand33);
        Command auto = new MultipleCommand(print1,new MultipleCommand(print2,print3));

        CommandScheduler scheduler = new CommandScheduler(null, printSub1, printSub2, printSub3);

        scheduler.initAuto();
        scheduler.forceCommand(auto);

        while (!scheduler.isEmpty()) {
            scheduler.run();
        }

        scheduler.shutdown();
    }
}
