package org.firstinspires.ftc.teamcode.visionPipelines;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.FOV;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.getCamWidth;
import static org.firstinspires.ftc.teamcode.visionPipelines.ObjectProc.poleDiameter;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Size;

public class pole {
    public Size pos;
    public Size size;
    public double perimeter;
    public double ratio;
    public double angle;
    public MatOfPoint contour;
    public boolean isReal;

    public pole(Size pos, Size size, double perimeter, double ratio,double angle, MatOfPoint contour, boolean isReal) {
        this.pos = pos;
        this.size = size;
        this.perimeter = perimeter;
        this.ratio = ratio;
        this.angle = angle;
        this.contour = contour;
        this.isReal = isReal;
    }
    public double getAngle() {
        return this.pos.width*(FOV/getCamWidth());
    }

    public double getDistance(){
        return poleDiameter/Math.sin( .5 * this.pos.width * FOV / getCamWidth() );
    }
}