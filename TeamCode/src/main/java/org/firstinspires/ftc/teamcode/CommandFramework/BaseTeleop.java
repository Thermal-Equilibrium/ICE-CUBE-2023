package org.firstinspires.ftc.teamcode.CommandFramework;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Utils.Team;

public abstract class BaseTeleop extends LinearOpMode {

    protected Robot robot;
    public final Pose2d initialPose = new Pose2d(-36, 66.5,Math.toRadians(-90)); // new Pose2d( -36, 63.5, Math.toRadians(-90));


    @Override
    public void runOpMode() {
        PhotonCore.enable();

        robot = new Robot(hardwareMap, Robot.OpMode.Auto, gamepad1, gamepad2, getTeam());
        robot.drivetrain.setPose(initialPose);

        waitForStart();

        robot.getScheduler().forceCommand(setupTeleop(robot.getScheduler()));

        while(opModeIsActive())
            robot.update();
    }

    /**
     * This method is called when the opmode is started. It should return a command that will be run
     * until the opmode is stopped.
     * @param scheduler The scheduler that will be used to run the command.
     * @return The command that will be run until the opmode is stopped.
     */
    public abstract Command setupTeleop(CommandScheduler scheduler);

    public Team getTeam() {
        return Team.NOT_ASSIGNED;
    }
}
