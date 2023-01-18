package org.firstinspires.ftc.teamcode.visionPipelines;

import org.opencv.core.Point;

public class VisionBasedPosition {
    public double distance;
    public double angle;

    public VisionBasedPosition(double distance, double angle) {
        this.distance = distance;
        this.angle = angle;
    }

}
