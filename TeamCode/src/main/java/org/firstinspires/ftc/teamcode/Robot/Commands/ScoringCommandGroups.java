package org.firstinspires.ftc.teamcode.Robot.Commands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveClaw;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveFlip;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveRotate;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveVerticalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Claw;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Flip;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Rotate;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;

public class ScoringCommandGroups {

	Claw claw;
	Flip flip;
	Rotate rotate;
	VerticalExtension extension;

	public ScoringCommandGroups(Claw claw, Flip flip, Rotate rotate, VerticalExtension extension) {
		this.claw = claw;
		this.flip = flip;
		this.rotate = rotate;
		this.extension = extension;
	}

	public Command setClaw(double position) {
		return new MoveClaw(claw,position);
	}

	public Command setFlip(double position) {
		return new MoveFlip(flip,position);
	}

	public Command setRotate(double position) {
		return new MoveRotate(rotate, position);
	}

	public Command setVertical(double position) {
		return new MoveVerticalExtension(extension,position);
	}

	public Command ready_for_intake_generic(double slide_height) {
		return setClaw(Claw.CLAW_CLOSED)
				.addNext(new MultipleCommand(setRotate(Rotate.ROTATE_PICKUP), setFlip(Flip.FLIP_PICKUP)))
				.addNext(setVertical(slide_height))
				.addNext(setClaw(Claw.CLAW_OPEN));
	}

	public Command ready_for_intake_auto(double slide_height) {
		return new MultipleCommand(setRotate(Rotate.ROTATE_PICKUP), setFlip(Flip.FLIP_PICKUP))
				.addNext(setClaw(Claw.CLAW_OPEN))
				.addNext(setVertical(slide_height));
	}

	public Command ready_for_intake_stack(double slide_height) {
		return new MultipleCommand(setVertical(slide_height), setRotate(Rotate.ROTATE_PICKUP), setFlip(Flip.FLIP_PICKUP))
				.addNext(setClaw(Claw.CLAW_OPEN));
	}

	public Command ready_for_intake() {
		return ready_for_intake_generic(VerticalExtension.IN_POSITION);
	}

	public Command grab_cone() {

		// only runs on manual release
		if (flip.is_folded) {
			claw.intentionalDropStart();
			return ready_for_intake();
		}

		return setClaw(Claw.CLAW_CLOSED)
				.addNext(setFlip(Flip.FLIP_FOLDED))
				.addNext(setVertical(VerticalExtension.GROUND_POSITION));
	}

	public Command sensor_cone_grab() {
		if (!flip.is_folded && !claw.is_intentionally_waiting) {
			// sensor grab only if its not flipped up and cone wasn't recently intentionally released
			return grab_cone();
		} else {
			// dont release cone cause already obtained
			return null;
		}
	}

	public Command grab_cone_auto() {
		return setClaw(Claw.CLAW_CLOSED)
				.addNext(
						new MultipleCommand(
								setFlip(Flip.FLIP_FOLDED),
								setVertical(VerticalExtension.MID_POSITION)
						)
				);
	}




	public Command scoring_height(double slide_pos) {

		if (slide_pos > VerticalExtension.GROUND_POSITION) {
			return new MultipleCommand(setVertical(slide_pos), setRotate(Rotate.ROTATE_DEPOSIT))
					.addNext(setFlip(Flip.FLIP_POLE_ALIGN));
		} else {
			return setVertical(slide_pos);
		}
	}

	public Command high() {
		return scoring_height(VerticalExtension.HIGH_POSITION);
	}
	public Command mid() {
		return scoring_height(VerticalExtension.MID_POSITION);
	}
	public Command low() {
		return scoring_height(VerticalExtension.LOW_POSITION);
	}

	// Auto stack manual height selection
	public Command stack5(double currTargetHeight) {
//		Dashboard.packet.put("bruh1", currTargetHeight);
//		Dashboard.packet.put("bruh2", VerticalExtension.CONE_5);
		// I don't know how to correctly check for is currently at picking up cone 5 state
		if (currTargetHeight == VerticalExtension.CONE_5) {
			return ready_for_intake();
		} else {
			return ready_for_intake_stack(VerticalExtension.CONE_5);
		}
	}
	public Command stack4() {
		return ready_for_intake_stack(VerticalExtension.CONE_4);
	}
	public Command stack3() {
		return ready_for_intake_stack(VerticalExtension.CONE_3);
	}
	public Command stack2() {
		return ready_for_intake_stack(VerticalExtension.CONE_2);
	}

	public Command ground() {
		return setFlip(Flip.FLIP_PICKUP).addNext(scoring_height(VerticalExtension.GROUND_POSITION));
	}

	public Command deposit_generic(Command intake_command) {
		return setFlip(Flip.FLIP_DEPOSIT)
				.addNext(setClaw(Claw.CLAW_OPEN))
				.addNext(setClaw(Claw.CLAW_TRANSFER_SAFE))
				.addNext(new MultipleCommand(setRotate(Rotate.ROTATE_PICKUP), setFlip(Flip.FLIP_PICKUP)))
				.addNext(intake_command);
	}


	public Command deposit_teleop() {
		return deposit_generic(ready_for_intake());
	}

}
