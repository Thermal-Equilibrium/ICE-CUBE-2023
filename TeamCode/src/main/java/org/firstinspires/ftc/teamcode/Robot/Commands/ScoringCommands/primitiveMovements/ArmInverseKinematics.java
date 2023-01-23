package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.Math.Kinematics.Intake3DKinematics;

public class ArmInverseKinematics extends Command {

    HorizontalExtension extension;
    Turret turret;
    double x;
    double y;
    double z;

    public ArmInverseKinematics(HorizontalExtension extension, Turret turret, double x, double y, double z) {
        this.extension = extension;
        this.turret = turret;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    @Override
    public void init() {

//        turret.setTurretAngle(Intake3DKinematics.getYawAngleToTarget(y, z)); not implemented yet, i assume worth did this
        turret.setArmDirect(Intake3DKinematics.getPitchAngleToTarget(z));
        extension.setTargetPosition(Intake3DKinematics.getHorizontalSlideExtensionToTarget(x, y, z));

    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean completed() {
        return false;
    }

    @Override
    public void shutdown() {

    }
}
