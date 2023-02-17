package org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.CancelableMoveArmDirect;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.BackCamera;
import org.firstinspires.ftc.teamcode.VisionUtils.Cone;

public class VisualIntakeStage1 extends Command {
    private final Turret turret;
    private final BackCamera backCamera;
    private final HorizontalExtension horizontalExtension;
    private boolean finished = false;


    public VisualIntakeStage1(Turret turret, BackCamera backCamera, HorizontalExtension horizontalExtension) {
        super(turret, backCamera);
        this.turret = turret;
        this.backCamera = backCamera;
        this.horizontalExtension = horizontalExtension;
    }

    @Override
    public void init() {
        if (this.backCamera.foundConestack) {
            this.horizontalExtension.setTargetPositionInches(this.backCamera.accumulatedConestackDistance / 2);
            this.turret.setBasedTurretPosition(this.backCamera.accumulatedConestackAngle);
        }
//        else {
//            this.horizontalExtension.setTargetPositionInches(4);
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
