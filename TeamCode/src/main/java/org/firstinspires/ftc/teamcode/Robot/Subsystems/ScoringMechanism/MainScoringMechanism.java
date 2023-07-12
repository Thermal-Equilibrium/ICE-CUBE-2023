package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class MainScoringMechanism extends Subsystem {

	public VerticalExtension verticalExtension = new VerticalExtension();
	public ArmSystem armSystem = new ArmSystem();


	MechanismStates state = MechanismStates.BEGIN;

	@Override
	public void initAuto(HardwareMap hwMap) {
		verticalExtension.initAuto(hwMap);
		armSystem.initAuto(hwMap);
	}

	@Override
	public void initTeleop(HardwareMap hwMap) {
		verticalExtension.initTeleop(hwMap);
		armSystem.initTeleop(hwMap);
	}

	@Override
	public void periodic() {
		updateMechanisms();
	}

	@Override
	public void shutdown() {

	}


	private void updateMechanisms() {
		verticalExtension.periodic();
		armSystem.periodic();
	}

	private void updateLogic() {
		// TODO: logic for moving states
	}

	public MechanismStates getState() {
		return state;
	}

	public enum MechanismStates {
		BEGIN,// everything in 18 inches, for match start
		CollectingTeleop,
		Collecting5,
		Collecting4,
		Collecting3,
		Collecting2,
		Collecting1,
		BringingTurretAndExtensionIn, // after in taking cone, turn the turret around and begin moving the horizontal Slides in
		TransferCone, // drop the cone onto th platform
		ReadyToDeposit, // cone is on platform, if necessary, horizontal extension is out of the way
		ReadyToDepositAuto, // same as the previous state but the horizontal slides are going all the way out instead of just partially out
		HIGH, // slide high
		MID, // slide MID
		GO_TO_LOW, // pickup cone from platform since our slides cannot on low or ground, we must just use the claw
		LOW, // in position on
	}


}
