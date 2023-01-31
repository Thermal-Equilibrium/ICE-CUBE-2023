package org.firstinspires.ftc.teamcode.VisionUtils;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.visionPipelines.ConeDetectionFast;
import org.opencv.core.Point;
import org.opencv.core.Size;

public class Cone {
    public enum Classification {
        PERFECT,
        FAR,
        CLOSE
    }

    public Size size;
    public VisionBasedPosition position;
    public Point top;
    public Classification classification;
    public boolean deadzoned;

    public Cone(Size size, VisionBasedPosition position, Point top){
        this.size = size;
        this.position = position;
        this.top = top;
        this.classify();
    }
    private void classify() {
        this.deadzoned = !(this.position.angle + this.position.cameraPosition.getHeading() <= Turret.MAX_SERVO_RADIANS) || !(this.position.angle + this.position.cameraPosition.getHeading() >= Turret.MIN_SERVO_RADIANS);
        if (this.position.distance < ConeDetectionFast.ConeConfig.perfectDistance - ConeDetectionFast.ConeConfig.perfectTolerance) {
            this.classification = Classification.CLOSE;
        }
        else if (this.position.distance > ConeDetectionFast.ConeConfig.perfectDistance + ConeDetectionFast.ConeConfig.perfectTolerance) {
            this.classification = Classification.FAR;
        }
        else {
            this.classification = Classification.PERFECT;
        }
    }
}
