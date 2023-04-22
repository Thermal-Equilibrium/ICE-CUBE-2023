package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;

public class AlignViaPole extends Command {
    Drivetrain drivetrain;
    double distanceFromPole = 3; //inches

    public AlignViaPole(Drivetrain drivetrain) {
        super(drivetrain);
        this.drivetrain = drivetrain;
    }

    @Override
    public void init() {
        Pose2d pose = drivetrain.getPose();
        // detect the nearest pole using the current position and the angle
        // the field is -70 to 70 in both axis
        // the poles are 23.3 inches apart

        // get the nearest center of pole to the robot
        double x = Math.round(pose.getX() / 23.3) * 23.3;
        double y = Math.round(pose.getY() / 23.3) * 23.3;

        // get the angle of the robot
        double robotAngle = pose.getHeading();

        // offset the robot position by the distance from the pole
        double xOff = x + distanceFromPole * Math.cos(robotAngle);
        double yOff = y + distanceFromPole * Math.sin(robotAngle);

        // update robot pose
        drivetrain.setPose(new Pose2d(xOff, yOff, robotAngle));
    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean completed() {
        return true;
    }

    @Override
    public void shutdown() {

    }
}
