package org.firstinspires.ftc.teamcode.CommandFramework;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RoadrunnerTrajectoryFollower;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.DelayedCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.RaceAction;
import org.firstinspires.ftc.teamcode.Robot.Commands.SafetyAlgorithms.FollowIfNeeded;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Purepursuit.Utils.Team;
import org.firstinspires.ftc.teamcode.VisionUtils.VisionMode;
import org.firstinspires.ftc.teamcode.visionPipelines.SleeveDetection;

import java.util.function.BooleanSupplier;

public abstract class BaseAuto extends LinearOpMode {

	protected Robot robot;
	protected TrajectoryBuilder trajectoryBuilder;

	protected SleeveDetection.ParkingPosition parkingPosition;

	public static double calculateTangent(Pose2d initialPosition, Pose2d finalPosition) {
		double xd = initialPosition.getX() - finalPosition.getX();
		double yd = initialPosition.getY() - finalPosition.getY();
		return Math.atan2(yd, xd) - Math.PI;
	}

	@Override
	public void runOpMode() {
		robot = new Robot(hardwareMap, Robot.OpMode.Auto, gamepad1, gamepad2);
		setRobotPosition();

		while (!isStopRequested() && !opModeIsActive() && opModeInInit()) {
			parkingPosition = robot.frontCamera.getParkingPosition();
			Dashboard.packet.put("current parking position is: ", parkingPosition);
			telemetry.addData("current parking position is: ", parkingPosition);
			telemetry.update();
		}

		robot.frontCamera.close();

		waitForStart();
		robot.getScheduler().forceCommand(setupAuto(robot.getScheduler()));

		while (opModeIsActive() && !isStopRequested()) {
			robot.update();
		}
		robot.shutdown();

	}

	public abstract Command setupAuto(CommandScheduler scheduler);

	public RoadrunnerTrajectoryFollower followRR(Trajectory trajectory) {
		return new RoadrunnerTrajectoryFollower(this.robot, trajectory, robot.dashboard);
	}

	public FollowIfNeeded followIfNeeded(Robot robot, Trajectory traj, BooleanSupplier externalStop) {
		return new FollowIfNeeded(this.robot,traj, this.robot.dashboard, externalStop);
	}


	public DelayedCommand delayCommand(double time, Command command) {
		return new DelayedCommand(time, command);
	}

	public MultipleCommand multiCommand(Command... commands) {
		return new MultipleCommand(commands);
	}

	public Delay wait(double seconds) {
		return new Delay(seconds);
	}

	public RaceAction raceCommand(Command... commands) {
		return new RaceAction(commands);
	}

	public void setRobotPosition() {

	}

	public Team getTeam() {
		return Team.NOT_ASSIGNED;
	}

	public VisionMode getVisionMode() { return VisionMode.SPICE; }
}
