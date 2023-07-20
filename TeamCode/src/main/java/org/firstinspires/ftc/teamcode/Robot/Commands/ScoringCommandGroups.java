package org.firstinspires.ftc.teamcode.Robot.Commands;

import android.util.Log;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
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

		// intentionally drop cone if manually triggered
		if (flip.is_folded) {
			claw.startIntentionalDrop();
			return ready_for_intake();
		}

		// Grab and move slides up if picking off stack
		if (extension.getSlideTargetPosition() >= VerticalExtension.CONE_2
				&& extension.getSlideTargetPosition() <= VerticalExtension.CONE_5) {
			return setClaw(Claw.CLAW_CLOSED)
					.addNext(setFlip(Flip.FLIP_FOLDED))
					.addNext(setVertical(VerticalExtension.LOW_POSITION));
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

	public Command scoring_front_height(double slide_pos) {
		return new MultipleCommand(setVertical(slide_pos), setRotate(Rotate.ROTATE_PICKUP))
				.addNext(setFlip(Flip.FLIP_FRONT_ALIGN));
	}

	public Command high() {
		return scoring_height(VerticalExtension.HIGH_POSITION);
	}
	public Command mid() { return scoring_height(VerticalExtension.MID_POSITION); }
	public Command low() {
		return scoring_height(VerticalExtension.LOW_POSITION);
	}
	public Command frontLow() { return scoring_front_height(VerticalExtension.LOW_POSITION); }

	// Auto stack manual height selection
	public Command stack5() {
		if (extension.getSlideTargetPosition() == VerticalExtension.CONE_5) {
			return ready_for_intake_stack(VerticalExtension.IN_POSITION);
		} else {
			return ready_for_intake_stack(VerticalExtension.CONE_5);
		}
	}
	public Command stack4() {
		if (extension.getSlideTargetPosition() == VerticalExtension.CONE_4) {
			return ready_for_intake_stack(VerticalExtension.IN_POSITION);
		} else {
			return ready_for_intake_stack(VerticalExtension.CONE_4);
		}
	}
	public Command stack3() {
		if (extension.getSlideTargetPosition() == VerticalExtension.CONE_3) {
			return ready_for_intake_stack(VerticalExtension.IN_POSITION);
		} else {
			return ready_for_intake_stack(VerticalExtension.CONE_3);
		}
	}
	public Command stack2() {
		if (extension.getSlideTargetPosition() == VerticalExtension.CONE_2) {
			return ready_for_intake_stack(VerticalExtension.IN_POSITION);
		} else {
			return ready_for_intake_stack(VerticalExtension.CONE_2);
		}
	}

	public Command ground() {
		claw.startIntentionalDrop();
		return setFlip(Flip.FLIP_PICKUP).addNext(scoring_height(VerticalExtension.GROUND_POSITION));
	}

	public Command deposit_generic(Command intake_command) {
		return setFlip(Flip.FLIP_DEPOSIT)
				.addNext(setClaw(Claw.CLAW_OPEN))
				.addNext(setClaw(Claw.CLAW_TRANSFER_SAFE))
				.addNext(new MultipleCommand(setRotate(Rotate.ROTATE_PICKUP), setFlip(Flip.FLIP_PICKUP)))
				.addNext(intake_command);
	}

	public Command deposit_ground() {
		return setClaw(Claw.CLAW_OPEN)
				.addNext(new MultipleCommand(setRotate(Rotate.ROTATE_PICKUP), setFlip(Flip.FLIP_PICKUP)))
				.addNext(setVertical(VerticalExtension.IN_POSITION));
	}

	public Command deposit_front(Command intake_command) {
		return setFlip(Flip.FLIP_PICKUP)
				.addNext(setClaw(Claw.CLAW_OPEN))
				.addNext(intake_command);
	}

	public Command deposit_teleop() {
		Log.i("Command", "Deposit Selection");
		if(extension.getSlideTargetPosition() == VerticalExtension.GROUND_POSITION) {
			Log.i("Command", "Deposit Ground");
			return deposit_ground();
		}

		if(flip.getPosition() == Flip.FLIP_FRONT_ALIGN) {
			Log.i("Command", "Deposit Front");
			return deposit_front(ready_for_intake_stack(VerticalExtension.IN_POSITION));
		}

		Log.i("Command", "Deposit Generic");
		return deposit_generic(ready_for_intake());
	}

}
