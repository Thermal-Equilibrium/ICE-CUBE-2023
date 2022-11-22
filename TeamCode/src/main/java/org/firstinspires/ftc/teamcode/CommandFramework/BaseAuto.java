package org.firstinspires.ftc.teamcode.CommandFramework;


import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RoadrunnerTrajectoryFollower;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.UltimateGoalMoment.FollowPath;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.UltimateGoalMoment.UGLqrPoseStabilization;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.DelayedCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.ActivateIntakeAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.DepositAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.GoToSafeHeight;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.GoToScore;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

public abstract class BaseAuto extends LinearOpMode {

	protected Robot robot;
	protected TrajectoryBuilder trajectoryBuilder;

	@Override
	public void runOpMode() {
		robot = new Robot(hardwareMap, Robot.OpMode.Auto, gamepad1, gamepad2);
		setRobotPosition();
		waitForStart();


		robot.getScheduler().forceCommand(setupAuto(robot.getScheduler()));

		while (opModeIsActive()) {
			robot.update();
		}
		robot.scoringMechanism.setWristToStow();

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

	public void setRobotPosition() {

	}


}
