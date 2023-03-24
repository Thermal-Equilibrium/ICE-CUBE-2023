package org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Math.Kinematics.IntakeKinematics;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.BackCamera;
import org.firstinspires.ftc.teamcode.VisionUtils.Cone;
import org.firstinspires.ftc.teamcode.VisionUtils.IntakeParameters;

import java.util.ArrayList;

public class GetIntakeParameters extends Command {
    private final Turret turret;
    private final BackCamera backCamera;
    private final HorizontalExtension horizontalExtension;
    private Cone target = null;

    private IntakeParameters intakeParameters;
    private int measurementNumber = 5;
    private int completedMeasures;
    private ArrayList<Double> angles;
    private ArrayList<Double> distances;
    private boolean isCompleted;

    public GetIntakeParameters(Turret turret, BackCamera backCamera, HorizontalExtension horizontalExtension) {
        super(turret, backCamera);
        this.turret = turret;
        this.backCamera = backCamera;
        this.horizontalExtension = horizontalExtension;
        this.intakeParameters = new IntakeParameters(false, 0, 0);
        this.angles = new ArrayList<>();
        this.distances = new ArrayList<>();
        this.isCompleted = false;
        this.completedMeasures = 0;
    }

    public IntakeParameters getIntakeParameters() {
        return this.intakeParameters;
    }

    @Override
    public void init() {
        this.measure();
    }

    @Override
    public void periodic() {
        this.measure();
    }

    @Override
    public boolean completed() {
        return this.isCompleted;
    }

    @Override
    public void shutdown() {
        turret.shutdown();
    }

    private void measure() {
        this.target = this.backCamera.getCone();
        boolean foundCone = false;
        double angle = 0;
        double extendDistance = 0;
        if (this.target != null) {
            foundCone = true;
            angle = IntakeKinematics.getTurretAngleToTarget(-1 * this.target.position.dx);
            extendDistance = IntakeKinematics.getHorizontalSlideExtensionToTarget(this.target.position.dy, -1 * this.target.position.dx, horizontalExtension.getSlidePositionInches());
            if (extendDistance <= 17) {
                if (angle < 0) {
                    angle += Math.PI * 2;
                }
            }
            else {
                foundCone = false;
            }
        }
        Dashboard.packet.put("foundCone",foundCone);
        Dashboard.packet.put("angle",angle);
        Dashboard.packet.put("extendDistance",foundCone);
        if (foundCone) {
            this.angles.add(angle);
            this.distances.add(extendDistance);
        }
        this.completedMeasures++;
//        this.intakeParameters.foundCone = foundCone;
//        this.intakeParameters.angle = angle;
//        this.intakeParameters.extendDistance = extendDistance;
        if (this.completedMeasures >= this.measurementNumber) {

//            this.intakeParameters.angle = angle;
            double totalAngle = 0;
            double totalDistance = 0;
            for (int i=0;i<this.angles.size();i++) {
                totalAngle += this.angles.get(i);
                totalDistance += this.distances.get(i);
            }
            this.intakeParameters.foundCone = this.angles.size() > 0;
            if (this.angles.size() > 1) {
                this.intakeParameters.angle = totalAngle / this.angles.size();
                this.intakeParameters.extendDistance = totalDistance / this.distances.size();
            }
            else {
                this.intakeParameters.angle = 0;
                this.intakeParameters.extendDistance = 0;
            }
            this.isCompleted = true;
        }
    }


}
