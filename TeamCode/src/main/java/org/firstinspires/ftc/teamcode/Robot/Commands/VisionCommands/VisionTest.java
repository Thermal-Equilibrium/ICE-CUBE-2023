package org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.visionPipelines.Cone;

public class VisionTest extends Command {
    Turret turret;
    Vision vision;
    Cone cone;
    boolean isCompleted;
    LowPassFilter filter;

    @Config
    public static class turretTest {
        public static double angle = .5;
    }

    public VisionTest(Turret turret, Vision vision) {
        super(turret, vision);
        this.turret = turret;
        this.vision = vision;
        this.isCompleted = false;
        this.filter = new LowPassFilter(0.85);
    }

    @Override
    public void init() {
        cone = vision.pipe.perfect;
        if (cone == null) {
            return;
        }
        turret.setTurretPositionSync(cone.servoAngle);
        Dashboard.packet.put("SERVO", cone.servoAngle);
    }

    @Override
    public void periodic() {
        cone = vision.pipe.perfect;
        if (cone == null) {
            return;
        }
        double angle = filter.estimate(cone.servoAngle);
        turret.setTurretPositionSync(angle);
        Dashboard.packet.put("SERVO", angle);
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
