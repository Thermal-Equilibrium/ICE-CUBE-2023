package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringMechanismCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Input;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.Utils.MathUtils;

// Turret control for teleop
// TODO: Add motion profiling
public class TurretControl extends Command {
    Turret turret;
    Input game_pad2;

    double turretRotationDeadBand = 0.1;
    double turretRotationScalingFactor = 1.0; // maybe incorporate dt?

    double armRotationDeadBand = 0.1;
    double armRotationScalingFactor = 1.0; // maybe incorporate dt?

    public TurretControl(Turret turret, Input game_pad2) {
        super(turret, game_pad2);
        this.turret = turret;
        this.game_pad2 = game_pad2;
    }

    @Override
    public void init() {
        turret.setTurretPositionSync(0.5);
        turret.setArmPositionSync(0.3);
        // I think we shouldn't do anything here since the turret is purely actuated by servos and we should probably conserve whatever their previous state was before this command was scheduled?
    }

    @Override
    public void periodic() {
        // turret rotation
        double turretRotationCommand = MathUtils.applyDeadBand(game_pad2.getStrafeJoystick(), turretRotationDeadBand) * 0.01;

        double deltaTurretPosition = turretRotationCommand * turretRotationScalingFactor;
        turret.setTurretPositionSync(turret.getTurretPosition() + deltaTurretPosition);

        // turret arm control
        double armRotationCommand = MathUtils.applyDeadBand(-game_pad2.getForwardJoystick(), armRotationDeadBand) * 0.01;

        double deltaArmPosition = armRotationCommand * armRotationScalingFactor;
        turret.setArmPositionSync(turret.getArmPosition() + deltaArmPosition);

        // TODO: Oh God someone help please I forgot how to do buttons correctly skull x3
        // also I don't know how ps4 controllers are laid out so adjust the specific button types to the drivers liking
        if (game_pad2.isTrianglePressed())
            turret.setClawGrabbing(Turret.ClawStates.Closed);
        else if (game_pad2.isCirclePressed())
            turret.setClawGrabbing(Turret.ClawStates.Open);
        else if (game_pad2.isSquarePressed())
            turret.setClawGrabbing(Turret.ClawStates.Transfer);
    }

    @Override
    public boolean completed() {
        return false;
    }

    @Override
    public void shutdown() {

    }
}
