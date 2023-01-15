package org.firstinspires.ftc.teamcode.visionPipelines;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;

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

    public Cone(MatOfPoint contour, VisionBasedPosition position, Point top, Classification classification, double cameraAngle){
        this.contour = contour;
        this.position = position;
        this.top = top;
        this.classification = classification;
        this.servoAngle = ServoMath.radiansToServo(position.angle-cameraAngle);
    }
}
