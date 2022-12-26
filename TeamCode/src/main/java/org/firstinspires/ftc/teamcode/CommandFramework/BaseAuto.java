package org.firstinspires.ftc.teamcode.CommandFramework;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RoadrunnerTrajectoryFollower;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.UltimateGoalMoment.FollowPath;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.UltimateGoalMoment.UGLqrPoseStabilization;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.DelayedCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.RelocalizeRobotFromPole;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.SetPoleContext;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.ActivateIntakeAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.DepositAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.GoToSafeHeight;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.GoToScore;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;
import org.firstinspires.ftc.teamcode.visionPipelines.SleeveDetection;

public abstract class BaseAuto extends LinearOpMode {

	protected Robot robot;
	protected TrajectoryBuilder trajectoryBuilder;

	protected SleeveDetection.ParkingPosition parkingPosition = SleeveDetection.ParkingPosition.CENTER;

	@Override
	public void runOpMode() {
		PhotonCore.enable();
		robot = new Robot(hardwareMap, Robot.OpMode.Auto, gamepad1, gamepad2);
		setRobotPosition();

		while (!isStopRequested() && !opModeIsActive() && opModeInInit()) {
			parkingPosition = robot.detectionSubsystem.getPosition();
			telemetry.addData("current parking position is: ", parkingPosition);
			telemetry.update();
		}

		robot.detectionSubsystem.destroy(); // TODO, Dear Worth this may cause issues...


		robot.getScheduler().forceCommand(setupAuto(robot.getScheduler()));

		while (opModeIsActive() && !isStopRequested()) {
			robot.update();
		}
//		robot.scoringMechanism.setWristToStow();
		robot.shutdown();

	}


	public abstract Command setupAuto(CommandScheduler scheduler);

	public RoadrunnerTrajectoryFollower followRR(Trajectory trajectory) {
		return new RoadrunnerTrajectoryFollower(this.robot, trajectory, robot.dashboard);
	}

	public UGLqrPoseStabilization goToLQR(Pose2d pose) {
		return new UGLqrPoseStabilization(robot.drivetrain, pose);
	}

	public FollowPath followPath(Pose2d... poses) {
		return new FollowPath(robot.drivetrain, poses);
	}

	public DelayedCommand delayCommand(double time, Command command) {
		return new DelayedCommand(time,command);
	}

	public MultipleCommand multiCommand(Command... commands) {
		return new MultipleCommand(commands);
	}

	public ActivateIntakeAuto intake() {
		return new ActivateIntakeAuto(robot.scoringMechanism);
	}


	public GoToScore goToScore() {
		return new GoToScore(robot.scoringMechanism, ScoringMechanism.States.HIGH);
	}

	public Delay wait(double seconds) {
		return new Delay(seconds);
	}

	public DepositAuto deposit() {
		return new DepositAuto(robot.scoringMechanism);
	}

	public GoToSafeHeight pregameScoring() {
		return new GoToSafeHeight(robot.scoringMechanism);
	}

	public RelocalizeRobotFromPole relocalizeRobot() {
		return new RelocalizeRobotFromPole(robot.distanceSensor);
	}

	public SetPoleContext getPoleContextualPosition() {
		return new SetPoleContext(robot.distanceSensor);
	}

	public void setRobotPosition() {

	}

	public static double calculateTangent(Pose2d initialPosition, Pose2d finalPosition) {
		double xd = initialPosition.getX() - finalPosition.getX();
		double yd = initialPosition.getY() - finalPosition.getY();
		return Math.atan2(yd,xd) - Math.PI;
	}


}
