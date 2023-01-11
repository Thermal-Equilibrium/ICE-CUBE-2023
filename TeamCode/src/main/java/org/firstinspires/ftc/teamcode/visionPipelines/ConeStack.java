package org.firstinspires.ftc.teamcode.visionPipelines;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;

public class ConeStack extends Cone{
    public ConeStack(MatOfPoint contour, VisionBasedPosition position, Point top, Classification classification) {
        super(contour,position,top,classification);
    }
    public static ConeStack fromCone(Cone cone) {
        return new ConeStack(cone.contour,cone.position,cone.top,cone.classification);
    }

}
