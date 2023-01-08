package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;


import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Input;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Utils.MathUtils;

public class RobotRelative extends Command {


    Drivetrain drivetrain;
    HorizontalExtension extension;
    Input game_pad1;
    protected boolean isBoostAppropriate = false;

    double strafe_dead_band = 0.1;

    public RobotRelative(Robot robot, Input game_pad1) {
        super(robot.drivetrain, game_pad1);
        this.drivetrain = robot.drivetrain;
        this.game_pad1 = game_pad1;
        this.extension = robot.scoringMechanism.horizontalExtension;

    }

    @Override
    public void init() {

    }

    @Override
    public void periodic() {


        double scalar = 1;

        double x;
        double y;
        double turn;
        y = game_pad1.getStrafeJoystick();
        x = game_pad1.getForwardJoystick();
        turn = game_pad1.getTurnJoystick();

        if (this.extension.getSlidePosition() > 200) {
            turn *= 0.5;
        }

        y = MathUtils.applyDeadBand(y, strafe_dead_band);

        Pose2d powers = new Pose2d(x * scalar,y * scalar,turn * scalar);


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
