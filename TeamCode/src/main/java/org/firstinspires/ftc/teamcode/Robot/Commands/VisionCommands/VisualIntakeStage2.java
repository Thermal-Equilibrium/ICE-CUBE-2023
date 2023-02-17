package org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.BackCamera;

public class VisualIntakeStage2 extends Command {
    private final Turret turret;
    private final BackCamera backCamera;
    private final HorizontalExtension horizontalExtension;
    private boolean finished = false;


    public VisualIntakeStage2(Turret turret, BackCamera backCamera, HorizontalExtension horizontalExtension) {
        super(turret, backCamera);
        this.turret = turret;
        this.backCamera = backCamera;
        this.horizontalExtension = horizontalExtension;
    }

    @Override
    public void init() {
        if (this.backCamera.foundConestack) {
            this.horizontalExtension.setTargetPositionInches(this.backCamera.accumulatedConestackDistance);
            this.turret.setBasedTurretPosition(this.backCamera.accumulatedConestackAngle);
        }
//        else {
//            this.horizontalExtension.setTargetPositionInches(5);
//            this.turret.setBasedTurretPosition(0);
//        }
    }

    @Override
    public void periodic() {
        if (horizontalExtension.isMovementFinished()) {
            this.finished = true;
        }
    }

    @Override
    public boolean completed() {
        return this.finished;
    }

    @Override
    public void shutdown() {
        turret.shutdown();
    }


}
