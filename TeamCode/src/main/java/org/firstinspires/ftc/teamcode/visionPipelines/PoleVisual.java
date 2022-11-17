package org.firstinspires.ftc.teamcode.visionPipelines;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.FOV;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.getCamWidth;
import static org.firstinspires.ftc.teamcode.visionPipelines.ObjectProc.poleDiameter;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Size;

public class PoleVisual {
    public Size pos;
    public Size size;
    public double perimeter;
    public double ratio;
    public double verticalAngle;
    public MatOfPoint contour;
    public boolean isReal;
    public double angle;
    public double distance;
    public Touching touching;


    public PoleVisual(Size pos, Size size, double perimeter, double ratio, double verticalAngle, MatOfPoint contour, boolean isReal, Touching touching) {
        this.pos = pos;
        this.size = size;
        this.perimeter = perimeter;
        this.ratio = ratio;
        this.verticalAngle = angle;
        this.contour = contour;
        this.isReal = isReal;
        this.touching = touching;

        this.angle = this.getAngle();
        this.distance = this.getDistance();

    }
    public double getAngle() {
        return this.pos.width*(FOV/getCamWidth());
    }

    public double getDistance(){
        return poleDiameter/(2*Math.sin( .5 * this.size.width * FOV / getCamWidth()));
    }
}