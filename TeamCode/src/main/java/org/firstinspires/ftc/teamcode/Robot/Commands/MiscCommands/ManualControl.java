package org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret.ArmStates.TRANSFER_SAFE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;



public class ManualControl extends Command {
    @Config
    public static class ManualControls {
        public static double horizontalExtensionInches = 0;
        public static double turretAngle = 180;
        public static int armPosition = 4;
        public static boolean clawGrabbing = false;
    }
    private final Turret turret;
    private final HorizontalExtension horizontalExtension;


    public ManualControl(Turret turret, HorizontalExtension horizontalExtension) {
        super(turret, horizontalExtension);
        this.turret = turret;
        this.horizontalExtension = horizontalExtension;
    }

    @Override
    public void init() {
        this.horizontalExtension.setTargetPositionInches(ManualControls.horizontalExtensionInches);
        this.turret.setBasedTurretPosition(Math.toRadians(ManualControls.turretAngle));
        if (ManualControls.armPosition == 0) this.turret.setArm(Turret.ArmStates.DOWN);
        else if (ManualControls.armPosition == 1) this.turret.setArm(Turret.ArmStates.LOW_SCORING);
        else if (ManualControls.armPosition == 2) this.turret.setArm(Turret.ArmStates.TRANSFER);
        else if (ManualControls.armPosition == 3) this.turret.setArm(TRANSFER_SAFE);
        if (ManualControls.clawGrabbing) this.turret.setClawGrabbing(Turret.ClawStates.Closed);
        else this.turret.setClawGrabbing(Turret.ClawStates.Open);
    }

    @Override
    public void periodic() {
        this.horizontalExtension.setTargetPositionInches(ManualControls.horizontalExtensionInches);
        this.turret.setBasedTurretPosition(Math.toRadians(ManualControls.turretAngle));
        if (ManualControls.armPosition == 0) this.turret.setArm(Turret.ArmStates.DOWN);
        else if (ManualControls.armPosition == 1) this.turret.setArm(Turret.ArmStates.LOW_SCORING);
        else if (ManualControls.armPosition == 2) this.turret.setArm(Turret.ArmStates.TRANSFER);
        else if (ManualControls.armPosition == 3) this.turret.setArm(TRANSFER_SAFE);
        if (ManualControls.clawGrabbing) this.turret.setClawGrabbing(Turret.ClawStates.Closed);
        else this.turret.setClawGrabbing(Turret.ClawStates.Open);
    }

    @Override
    public boolean completed() {
        return false;
    }

    @Override
    public void shutdown() {
        turret.shutdown();
        horizontalExtension.shutdown();
    }


}
