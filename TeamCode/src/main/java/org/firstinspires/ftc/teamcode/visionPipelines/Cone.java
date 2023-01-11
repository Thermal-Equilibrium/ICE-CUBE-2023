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

    public Cone(MatOfPoint contour, VisionBasedPosition position, Point top, Classification classification){
        this.contour = contour;
        this.position = position;
        this.top = top;
        this.classification = classification;
        this.servoAngle  = radianToServo(position.angle);
    }

    private double radianToServo(double radians) {
        return 0 - .1 *  radians / Math.toRadians(30);
    }


}
