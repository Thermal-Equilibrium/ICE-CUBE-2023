package org.firstinspires.ftc.teamcode.Robot.Commands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveClaw;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveFlip;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveRotate;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveVerticalExtension;
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

	public Command ready_for_intake() {
		return setClaw(Claw.CLAW_CLOSED)
				.addNext(new MultipleCommand(setRotate(Rotate.ROTATE_PICKUP), setFlip(Flip.FLIP_PICKUP)))
				.addNext(setVertical(VerticalExtension.IN_POSITION))
				.addNext(setClaw(Claw.CLAW_OPEN));
	}

	public Command grab_cone() {

		// try again
		if (flip.is_folded) {
			return ready_for_intake();
		}

		return setClaw(Claw.CLAW_CLOSED)
				.addNext(setFlip(Flip.FLIP_FOLDED))
				.addNext(setVertical(VerticalExtension.GROUND_POSITION));
	}



	public Command scoring_height(double slide_pos) {

		if (slide_pos != VerticalExtension.GROUND_POSITION) {
			return new MultipleCommand(setVertical(slide_pos), new Delay(0.3).addNext(setRotate(Rotate.ROTATE_DEPOSIT)))
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
	public Command ground() {
		return setFlip(Flip.FLIP_PICKUP).addNext(scoring_height(VerticalExtension.GROUND_POSITION));
	}
	public Command deposit() {
		return setFlip(Flip.FLIP_DEPOSIT)
				.addNext(setClaw(Claw.CLAW_OPEN))
				.addNext(setClaw(Claw.CLAW_TRANSFER_SAFE))
				.addNext(new MultipleCommand(setRotate(Rotate.ROTATE_PICKUP), setFlip(Flip.FLIP_PICKUP)))
				.addNext(ready_for_intake());
	}

}
