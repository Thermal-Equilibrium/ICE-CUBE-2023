package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.NullCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.RunCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveArm;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveArmDirect;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveClaw;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveHorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveTurret;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveTurretAsync;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveVerticalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.MainScoringMechanism;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;

public class ScoringCommandGroups {

	Turret turret;
	VerticalExtension verticalExtension;
	HorizontalExtension horizontalExtension;

	public ScoringCommandGroups(MainScoringMechanism mechanism) {
		this.horizontalExtension = mechanism.horizontalExtension;
		this.verticalExtension = mechanism.verticalExtension;
		this.turret = mechanism.turret;
	}

	// near straight but tilted to the left
	public Command moveToIntakingLeft() {
		return moveHorizontalExtension(HorizontalExtension.EXTENSION1)
				.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE))
				.addNext(moveTurret(Turret.TurretStates.Slight_LEFT))
				.addNext(new MultipleCommand(moveArm(Turret.ArmStates.DOWN), openClaw()));
	}

	public Command moveToIntakingLeftWithDeposit() {
		return moveTurretAsync(Turret.TurretStates.FAR_LEFT)
				.addNext(new MultipleCommand(moveHorizontalExtension(HorizontalExtension.autoExtension),
							moveVerticalExtension(VerticalExtension.HIGH_POSITION)))
				.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE))
				.addNext(new MultipleCommand(moveArm(Turret.ArmStates.DOWN), openClaw()));
	}
	public Command moveToIntakingRight() {
		return moveHorizontalExtension(HorizontalExtension.EXTENSION1)
				.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE))
				.addNext(moveTurret(Turret.TurretStates.Slight_RIGHT_AUTO))
				.addNext(new MultipleCommand(moveArm(Turret.ArmStates.DOWN), openClaw()));
	}

	public int currentCone = 5;
	double[] armConeHeights = {0.08, 0.11, 0.1357, 0.1632, 0.1978};

	public Command moveToIntakingRightAuto() {
		currentCone--;

//		return moveHorizontalExtension(HorizontalExtension.EXTENSION3)
//				.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE))
//				.addNext(moveTurret(Turret.TurretStates.Slight_RIGHT_AUTO))
//				.addNext(new MultipleCommand(moveArmDirect(-0.03 + armConeHeights[currentCone]), openClaw()));

		return moveArm(Turret.ArmStates.TRANSFER_SAFE)
				.addNext(moveTurret(Turret.TurretStates.Slight_RIGHT_AUTO))
				.addNext(new MultipleCommand(moveArmDirect(-0 + armConeHeights[currentCone]), openClaw()));
	}

	// very far to the left, in order to place near by
	public Command moveToIntakingLeftClosePole() {
		return moveArm(Turret.ArmStates.TRANSFER_SAFE)
				.addNext(moveHorizontalExtension(HorizontalExtension.PRE_EMPTIVE_EXTEND))
				.addNext(moveTurret(Turret.TurretStates.FAR_LEFT))//new MultipleCommand(moveArm(Turret.ArmStates.TRANSFER_SAFE),
				//moveHorizontalExtension(HorizontalExtension.PRE_EMPTIVE_EXTEND),
				//moveTurret(Turret.TurretStates.FAR_LEFT))
				.addNext(new MultipleCommand(moveArm(Turret.ArmStates.DOWN), openClaw()))
				.addNext(moveHorizontalExtension(HorizontalExtension.EXTENSION2));
	}

	public Command moveToIntakingRightClosePole() {
		Command command = new MultipleCommand(moveArm(Turret.ArmStates.TRANSFER_SAFE),
				moveHorizontalExtension(HorizontalExtension.PRE_EMPTIVE_EXTEND),
				moveTurret(Turret.TurretStates.FAR_RIGHT))
				.addNext(new Delay(0.1))
				.addNext(new MultipleCommand(moveArm(Turret.ArmStates.DOWN), openClaw()))
				.addNext(moveHorizontalExtension(HorizontalExtension.EXTENSION2));
		return new RunCommand(() -> command);
	}

	public Command syncedScoringCollecting() {
		Command depositCommand = moveVerticalExtension(VerticalExtension.HIGH_POSITION);
		Command intakeCommand = moveToIntakingLeft();
		return intakeCommand.addNext(depositCommand);
	}


	public Command collectCone() {

		if (verticalExtension.getSlidePosition() > 50) {
			return new NullCommand();
		}

		return grabCone()
				.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE))
				.addNext(moveTurret(Turret.TurretStates.TRANSFER))
				.addNext(moveHorizontalExtension(HorizontalExtension.IN_POSITION))
				.addNext(moveArm(Turret.ArmStates.TRANSFER))
				.addNext(releaseCone())
				.addNext(new Delay(0.2))
				.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE));
	}
	public Command collectConeAuto() {

		return grabCone()
				.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE))
				.addNext(moveTurret(Turret.TurretStates.TRANSFER))
				.addNext(moveHorizontalExtension(HorizontalExtension.IN_POSITION))
				.addNext(moveArm(Turret.ArmStates.TRANSFER))
				.addNext(releaseCone())
				.addNext(new Delay(0.2))
				.addNext(moveArm(Turret.ArmStates.TRANSFER_SAFE));
	}

	public Command grabCone() {
		return moveClaw(Turret.ClawStates.Closed);
	}

	public Command releaseCone() {
		return moveClaw(Turret.ClawStates.Transfer);
	}

	public Command openClaw() {
		return moveClaw(Turret.ClawStates.Open);
	}

	public Command moveToLowGoalScoring() {
		return moveClaw(Turret.ClawStates.Closed)
				.addNext(moveArm(Turret.ArmStates.LOW_SCORING))
				.addNext(moveHorizontalExtension(HorizontalExtension.IN_POSITION));
	}

	// Move vertical extension down. If it is already going down, open the claw
	public Command moveVerticalExtensionDownOrReleaseClaw() {
		if (verticalExtension.getSlideTargetPosition() == VerticalExtension.IN_POSITION) {
			return moveClaw(Turret.ClawStates.Open);
		} else {
			return moveVerticalExtension(VerticalExtension.IN_POSITION);
		}
	}

	public MoveArm moveArm(Turret.ArmStates armStates) {
		return new MoveArm(turret,armStates);
	}
	public MoveArmDirect moveArmDirect(double position) {
		return new MoveArmDirect(turret, position);
	}
	public MoveClaw moveClaw(Turret.ClawStates clawStates) {
		return new MoveClaw(turret,clawStates);
	}
	public MoveTurret moveTurret(Turret.TurretStates turretStates) {
		return new MoveTurret(turret,turretStates);
	}
	public MoveTurret moveTurretAsync(Turret.TurretStates turretStates) {
		return new MoveTurretAsync(turret,turretStates);
	}
	public MoveHorizontalExtension moveHorizontalExtension(double position) {
		return new MoveHorizontalExtension(horizontalExtension,position);
	}
	public MoveVerticalExtension moveVerticalExtension(double position) {
		return new MoveVerticalExtension(verticalExtension,position);
	}
}
