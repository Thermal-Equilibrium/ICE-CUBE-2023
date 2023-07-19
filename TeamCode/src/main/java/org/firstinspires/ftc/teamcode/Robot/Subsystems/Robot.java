package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Claw;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Flip;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Rotate;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.FrontCamera;
import org.firstinspires.ftc.teamcode.Simulation.TestCommandsSubsystems.PrintSubsystem1;

import java.util.ArrayList;

public class Robot {


	public Dashboard dashboard = new Dashboard();
	public Input gamepad1;
	public Input gamepad2;
	public Drivetrain drivetrain = new Drivetrain();
	public VerticalExtension extension = new VerticalExtension();
	public Claw claw = new Claw();
	public Flip flip = new Flip();
	public Rotate rotate = new Rotate();
	public FieldMap field = new FieldMap();
	public FrontCamera frontCamera;
	public ConeSensors coneSensors;
	// print subsystem for testing
	public PrintSubsystem1 print = new PrintSubsystem1();
	protected CommandScheduler scheduler;
	ArrayList<LynxModule> modules = new ArrayList<>();

	public Robot(HardwareMap hwMap, OpMode opMode, Gamepad gamepad1, Gamepad gamepad2) {
		frontCamera = new FrontCamera();
		scheduler = new CommandScheduler(hwMap, drivetrain, dashboard, field,print,extension,claw,flip,rotate);
		this.gamepad1 = new Input(gamepad1, scheduler);
		this.gamepad2 = new Input(gamepad2, scheduler);
		coneSensors = new ConeSensors(scheduler);
		scheduler.appendSubsystem(coneSensors);

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
		Dashboard.packet.put("field delay", field.getDelayLength());

//		Dashboard.packet.put("left stick x", gamepad1.getLeft_stick_x());
//		Dashboard.packet.put("left stick y", gamepad1.getLeft_stick_y());
//		Dashboard.packet.put("right stick x", gamepad1.getRight_stick_x());
//		Dashboard.packet.put("right stick y", gamepad1.getRight_stick_y());
//
//		Dashboard.packet.put("forward joystick", gamepad1.getForwardJoystick());
//		Dashboard.packet.put("strafe joystick", gamepad1.getStrafeJoystick());
//		Dashboard.packet.put("turn joystick", gamepad1.getTurnJoystick());
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
