package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.ArmSystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;

public class MoveClawOnceSlidesDown extends Command {
	double delayS = 0.20;

	ArmSystem armSystem;
	ArmSystem.ClawStates clawState;
	VerticalExtension verticalExtension;
	ElapsedTime timer = new ElapsedTime();

	public MoveClawOnceSlidesDown(ArmSystem armSystem, VerticalExtension verticalExtension, ArmSystem.ClawStates clawState) {
		this.armSystem = armSystem;
		this.clawState = clawState;
		this.verticalExtension = verticalExtension;
	}

	@Override
	public void init() {
		timer.reset();
	}

	@Override
	public void periodic() {
		if (verticalExtension.slideIsDown()) {
			armSystem.setClawGrabbing(clawState);
		} else {
			timer.reset();
		}
 	}

	@Override
	public boolean completed() {
		return timer.seconds() > delayS && verticalExtension.slideIsDown();
	}

	@Override
	public void shutdown() {

	}
}
