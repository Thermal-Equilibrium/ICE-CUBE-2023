package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.MainScoringMechanism;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.BackCamera;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.FrontCamera;
import org.firstinspires.ftc.teamcode.Simulation.TestCommandsSubsystems.PrintSubsystem1;
import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.teamcode.VisionUtils.VisionMode;

import java.util.ArrayList;

public class Robot {


	public Dashboard dashboard = new Dashboard();
	public Input gamepad1;
	public Input gamepad2;
	public Drivetrain drivetrain = new Drivetrain();
	public MainScoringMechanism scoringMechanism = new MainScoringMechanism();
	public FieldMap field = new FieldMap();
	public FrontCamera frontCamera;
	// print subsystem for testing
	public PrintSubsystem1 print = new PrintSubsystem1();
	protected CommandScheduler scheduler;
	ArrayList<LynxModule> modules = new ArrayList<>();

	public Robot(HardwareMap hwMap, OpMode opMode, Gamepad gamepad1, Gamepad gamepad2) {
		frontCamera = new FrontCamera();
		scheduler = new CommandScheduler(hwMap, drivetrain, dashboard, field,print);
		this.gamepad1 = new Input(gamepad1, scheduler);
		this.gamepad2 = new Input(gamepad2, scheduler);

		if (opMode.equals(OpMode.Auto)) {
			scheduler.initAuto();
		} else if (opMode.equals(OpMode.Teleop)) {
			scheduler.initTeleop();
		}
		for (LynxModule module : hwMap.getAll(LynxModule.class)) {
			module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
			modules.add(module);
		}
	}

	public void update() {
		for (LynxModule module : modules) {
			module.clearBulkCache();
		}
		updateInput();
		scheduler.run();
		Dashboard.packet.put("Dash Delay", dashboard.getDelayLength());
		Dashboard.packet.put("game1 Delay", gamepad1.getDelayLength());
		Dashboard.packet.put("game2 Delay", gamepad2.getDelayLength());
		Dashboard.packet.put("drivetrain Delay", drivetrain.getDelayLength());
		Dashboard.packet.put("scoring Delay", scoringMechanism.getDelayLength());
		Dashboard.packet.put("distanceSensor Delay", scoringMechanism.getDelayLength());
		Dashboard.packet.put("field delay", field.getDelayLength());
	}

	public void shutdown() {
		scheduler.shutdown();
	}

	public void updateInput() {
		gamepad1.periodic();
		gamepad2.periodic();
	}

	public CommandScheduler getScheduler() {
		return scheduler;
	}

	public enum OpMode {
		Auto,
		Teleop
	}
}
