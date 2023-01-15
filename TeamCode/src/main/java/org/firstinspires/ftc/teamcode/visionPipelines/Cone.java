package org.firstinspires.ftc.teamcode.visionPipelines;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

public class Cone {
    public enum Classification {
        PERFECT,
        FAR,
        CLOSE,
        DEADZONE
    }
    public MatOfPoint contour;
    public VisionBasedPosition position;
    public Point top;
    public Classification classification;
    public double servoAngle;

    public Cone(MatOfPoint contour, VisionBasedPosition position, Point top, double cameraAngle){
        this.contour = contour;
        this.position = position;
        this.top = top;
        this.servoAngle = ServoMath.radiansToServo(position.angle-cameraAngle);
    }
    private void classify() {
        Cone.Classification classification;
        if (this.servoAngle <= 1 && this.servoAngle >= 0){
            this.classification = Cone.Classification.DEADZONE;
        }
        else if (this.position.distance < Optimized.VisionConfig.perfectDistance - Optimized.VisionConfig.perfectTolerance) {
            this.classification = Cone.Classification.CLOSE;
        }
        else if (this.position.distance > Optimized.VisionConfig.perfectDistance + Optimized.VisionConfig.perfectTolerance) {
            this.classification = Cone.Classification.FAR;
        }
        else {
            this.classification = Cone.Classification.PERFECT;
        }
    }
}
