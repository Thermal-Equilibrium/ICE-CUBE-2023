package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import static org.firstinspires.ftc.teamcode.RR_quickstart.trajectorysequence.TrajectorySequenceRunner.POSE_HISTORY_LIMIT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.RR_quickstart.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;

import java.util.LinkedList;

public class RoadrunnerTrajectoryFollower extends Command {


    private final Robot robot;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();
    Trajectory traj;
    Pose2d[] positions;
    int pathPrecision = 50; // number of points to graph in the path.


    public RoadrunnerTrajectoryFollower(Robot robot, Trajectory traj, Dashboard dashboard) {
        super(robot.drivetrain);
        this.robot = robot;
        this.traj = traj;
        this.robot.drivetrain.setTrajectoryTracking(true);
    }


    @Override
    public void init() {
        robot.drivetrain.followTrajectory(traj);

    }

    @Override
    public void periodic() {

        try {
            poseHistory.add(robot.drivetrain.getPose());
            while (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
                poseHistory.removeFirst();
            }
            DashboardUtil.drawSampledPath(Dashboard.packet.fieldOverlay(), traj.getPath());
            DashboardUtil.drawPoseHistory(Dashboard.packet.fieldOverlay(), poseHistory);
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("thing to draw trajectory broke lol, probably due to index bounds, don't worry about it. not that important ");
        }

    }

    @Override
    public boolean completed() {
        return !robot.drivetrain.isBusy();
    }

    @Override
    public void shutdown() {
        robot.drivetrain.shutdown();
    }
}
