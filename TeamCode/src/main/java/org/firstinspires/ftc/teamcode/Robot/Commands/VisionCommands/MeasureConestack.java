package org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Math.Kinematics.IntakeKinematics;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.CancelableMoveArmDirect;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.BackCamera;
import org.firstinspires.ftc.teamcode.VisionUtils.Cone;

public class MeasureConestack extends Command {
    private final Turret turret;
    private final BackCamera backCamera;
    private final HorizontalExtension horizontalExtension;
    private Cone target = null;
    private double angle;
    private double extendDistance;

    public MeasureConestack(Turret turret, BackCamera backCamera, HorizontalExtension horizontalExtension) {
        super(turret, backCamera);
        this.turret = turret;
        this.backCamera = backCamera;
        this.horizontalExtension = horizontalExtension;
    }

    @Override
    public void init() {
        this.target = this.backCamera.getCone();
        if (this.target != null) {
            this.backCamera.foundConestack = true;
            CancelableMoveArmDirect.cancelled = false;
            this.angle = IntakeKinematics.getTurretAngleToTarget(-1 * this.target.position.dx);
            this.extendDistance = IntakeKinematics.getHorizontalSlideExtensionToTarget(this.target.position.dy, -1 * this.target.position.dx, horizontalExtension.getSlidePositionInches());
            if (extendDistance <= 16) {
                if (this.angle < 0) {
                    this.angle += Math.PI * 2;
                }
                Dashboard.packet.put("1MEASURED ANGLE", angle);
                Dashboard.packet.put("1MEASURED DISTANCE", angle);
                this.backCamera.accumulatedConestackAngle = this.angle;
                this.backCamera.accumulatedConestackDistance =  this.extendDistance;
//                if (this.backCamera.accumulatedConestackDistance == 0) {
//                    this.backCamera.accumulatedConestackAngle = this.angle;
//                    this.backCamera.accumulatedConestackDistance =  this.extendDistance;
//                }
//                else {
//                    this.backCamera.accumulatedConestackAngle = (this.backCamera.accumulatedConestackAngle + this.angle) / 2;
//                    this.backCamera.accumulatedConestackDistance = (this.backCamera.accumulatedConestackDistance + this.extendDistance) / 2;
//                    }
                Dashboard.packet.put("1ACCUMULATED ANGLE", this.backCamera.accumulatedConestackAngle);
                Dashboard.packet.put("1ACCUMULATED DISTANCE", this.backCamera.accumulatedConestackDistance);

            }
        } else {
            this.backCamera.foundConestack = false;
            CancelableMoveArmDirect.cancelled = true;
        }
    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean completed() {
        return true;
    }

    @Override
    public void shutdown() {
        turret.shutdown();
    }


}
