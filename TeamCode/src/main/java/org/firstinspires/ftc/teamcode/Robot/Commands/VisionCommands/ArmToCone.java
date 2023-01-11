package org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.visionPipelines.Cone;

public class ArmToCone extends Command {
    Turret turret;
    Vision vision;
    Cone cone;
    boolean isCompleted;
    LowPassFilter filter;

    @Config
    public static class turretTest {
        public static double angle = .5;
    }

    public ArmToCone(Turret turret, Vision vision) {
        super(turret, vision);
        this.turret = turret;
        this.vision = vision;
        this.isCompleted = false;
    }

    @Override
    public void init() {
        cone = vision.pipe.theCone;
        if (cone == null) {
            return;
        }
        turret.setTurretPositionSync(cone.servoAngle);
        Dashboard.packet.put("SERVO", cone.servoAngle);
        this.isCompleted=true;

    }

    @Override
    public void periodic() {
        cone = vision.pipe.theCone;
        if (cone == null) {
            return;
        }
        turret.setTurretPositionSync(cone.servoAngle);
        Dashboard.packet.put("SERVO", cone.servoAngle);
        this.isCompleted=true;
    }

    @Override
    public boolean completed() {
        return this.isCompleted;
    }

    @Override
    public void shutdown() {
        turret.shutdown();
    }

}
