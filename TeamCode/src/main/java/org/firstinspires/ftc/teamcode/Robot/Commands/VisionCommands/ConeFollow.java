package org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.BackCamera;
import org.firstinspires.ftc.teamcode.VisionUtils.Cone;

public class ConeFollow extends Command {
    private Turret turret;
    private BackCamera backCamera;
    private Cone target = null;

    public ConeFollow(Turret turret, BackCamera backCamera) {
        super(turret, backCamera);
        this.turret = turret;
        this.backCamera = backCamera;
    }

    @Override
    public void init() {
        this.target = this.backCamera.getCone();
        if (this.target != null) {
            this.turret.setBasedTurretPosition(Math.toRadians(360) - this.target.position.angle);
            Dashboard.packet.put("ConeAngle", Math.toDegrees(Math.toRadians(360) - this.target.position.angle));
        }
    }

    @Override
    public void periodic() {
        this.target = this.backCamera.getCone();
        if (this.target != null) {
            this.turret.setBasedTurretPosition(Math.toRadians(360) - this.target.position.angle);
            Dashboard.packet.put("ConeAngle", Math.toDegrees(Math.toRadians(360) - this.target.position.angle));
        }
    }

    @Override
    public boolean completed() { return false; }

    @Override
    public void shutdown() {
        turret.shutdown();
    }


}
