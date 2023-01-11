package org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.visionPipelines.Cone;

public class ArmToCone extends Command {
    double delayS = 0.35;

    Turret turret;
    Vision vision;
    Cone cone;


    ElapsedTime timer = new ElapsedTime();

    @Config
    public static class turretTest {
        public static double angle = .5;
    }

    public ArmToCone(Turret turret, Vision vision) {
        super(turret, vision);
        this.turret = turret;
        this.vision = vision;
    }

    @Override
    public void init() {
        cone = vision.pipe.theCone;
        if (cone == null) {
            return;
        }
        turret.setTurretPositionSync(cone.servoAngle);
        Dashboard.packet.put("SERVO", cone.servoAngle);
        timer.reset();
    }

    @Override
    public void periodic() {
    }

    @Override
    public boolean completed() {
        return timer.seconds() > delayS;
    }

    @Override
    public void shutdown() {
        turret.shutdown();
    }

}
