package org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.VisionUtils.IntakeParameters;

public class VisualIntakeStage1 extends Command {
    private static final double INTAKE_STAGE_1_DISTANCE_PERCENT = 75; // the percent of the horizontal extension that will be done in stage 1, the rest will be in stage 2
    private final Turret turret;
    private final HorizontalExtension horizontalExtension;
    private boolean finished = false;
    private IntakeParameters intakeParameters;


    public VisualIntakeStage1(IntakeParameters intakeParameters, Turret turret, HorizontalExtension horizontalExtension) {
        super(turret, horizontalExtension);
        this.intakeParameters = intakeParameters;
        this.turret = turret;
        this.horizontalExtension = horizontalExtension;
    }

    @Override
    public void init() {
        if (this.intakeParameters.foundCone) {
            this.horizontalExtension.setTargetPositionInches(this.intakeParameters.extendDistance * (INTAKE_STAGE_1_DISTANCE_PERCENT / 100));
            this.turret.setBasedTurretPosition(this.intakeParameters.angle);
        }
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
