package org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.visionPipelines.Cone;
import org.firstinspires.ftc.teamcode.visionPipelines.Optimized;

public class TurretToConestack extends Command {
    @Override
    public void init() {

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

//    private Turret turret;
//    private Vision vision;
//    private boolean targetLock;
//    private boolean foundTarget = false;
//    private boolean isCompleted = false;
//    private Cone target = null;
//
//    public TurretToConestack(Turret turret, Vision vision, boolean targetLock) {
//        super(turret, vision);
//        this.turret = turret;
//        this.vision = vision;
//        this.targetLock = targetLock;
//    }
//
//    @Override
//    public void init() {
//        this.findTarget();
//    }
//
//    @Override
//    public void periodic() {
//        if (!this.foundTarget || !this.targetLock) this.findTarget();
//        if (Math.abs(this.turret.getTurretPosition() - this.target.servoAngle) <= .02) this.isCompleted = true;
//    }
//
//    @Override
//    public boolean completed() {
//        return this.isCompleted;
//    }
//
//    @Override
//    public void shutdown() {
//        turret.shutdown();
//    }
//
//    private void findTarget() {
//        assert this.vision.backCam.pipe instanceof Optimized;
//        this.target = ((Optimized) this.vision.backCam.pipe).conestackGuess;
//        if (this.target != null) {
//            this.foundTarget = true;
//            this.turret.setTurretPositionSync(this.target.servoAngle);
//            Dashboard.packet.put("SERVO", this.target.servoAngle);
//        }
//    }
}
