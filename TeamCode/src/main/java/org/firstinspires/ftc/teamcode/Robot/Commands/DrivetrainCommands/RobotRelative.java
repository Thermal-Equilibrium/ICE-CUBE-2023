package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;


import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Input;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

public class RobotRelative extends Command {


    Drivetrain drivetrain;
    ScoringMechanism mechanism;
    Input game_pad1;
    protected boolean isBoostAppropriate = false;

    double strafe_dead_band = 0.1;

    public RobotRelative(Robot robot, Input game_pad1) {
        super(robot.drivetrain, game_pad1);
        this.drivetrain = robot.drivetrain;
        this.mechanism = robot.scoringMechanism;;
        this.game_pad1 = game_pad1;

    }

    @Override
    public void init() {

    }

    @Override
    public void periodic() {


        double scalar = 1;
        ScoringMechanism.States state = mechanism.getState();
        if (state.equals(ScoringMechanism.States.HIGH)
        || state.equals(ScoringMechanism.States.GO_TO_HIGH) ||
        state.equals(ScoringMechanism.States.MID) ||
        state.equals(ScoringMechanism.States.GO_TO_MID) ||
        state.equals(ScoringMechanism.States.LOW) ||
        state.equals(ScoringMechanism.States.GO_TO_LOW) ||
        state.equals(ScoringMechanism.States.GO_TO_INTAKE)) {
            scalar = 0.7;
        }


        if (state.equals(ScoringMechanism.States.HIGH) || state.equals(ScoringMechanism.States.MID)) {
            isBoostAppropriate = true;
        }
        if (state.equals(ScoringMechanism.States.LOW)) {
            isBoostAppropriate = false;
        }

        double x;
        double y;
        double turn;
        y = game_pad1.getStrafeJoystick();
        x = game_pad1.getForwardJoystick();
        turn = game_pad1.getTurnJoystick();

        if (Math.abs(y) < strafe_dead_band) {
            y = 0;
        }

        Pose2d powers = new Pose2d(x * scalar,y * scalar,turn * scalar);

        if (state.equals(ScoringMechanism.States.DEPOSIT) && isBoostAppropriate) {
            powers = new Pose2d(0.7,0,turn * scalar);
        }

        drivetrain.robotRelative(powers);


    }

    @Override
    public boolean completed() {
        return false;
    }

    @Override
    public void shutdown() {
        drivetrain.shutdown();
    }
}
