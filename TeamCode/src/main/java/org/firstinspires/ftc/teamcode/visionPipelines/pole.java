package org.firstinspires.ftc.teamcode.visionPipelines;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.FOV;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.getCamWidth;
import static org.firstinspires.ftc.teamcode.visionPipelines.ObjectProc.poleDiameter;

import org.firstinspires.ftc.teamcode.Math.TheArcaneConceptThatIsTurningInPlace.Heading;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Size;

public class pole {
    public Size pos;
    public Size size;
    public double perimeter;
    public double ratio;
    public double verticality;
    public MatOfPoint contour;
    public boolean isReal;
    public double angle;
    public double distance;


    public pole(Size pos, Size size, double perimeter, double ratio,double verticality, MatOfPoint contour, boolean isReal) {
        this.pos = pos;
        this.size = size;
        this.perimeter = perimeter;
        this.ratio = ratio;
        this.verticality = angle;
        this.contour = contour;
        this.isReal = isReal;
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