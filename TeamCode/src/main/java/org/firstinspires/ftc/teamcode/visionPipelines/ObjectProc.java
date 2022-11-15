package org.firstinspires.ftc.teamcode.visionPipelines;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.FOV;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.getCamWidth;
import static org.firstinspires.ftc.teamcode.visionPipelines.PolePipe.draw;

import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.Math.TheArcaneConceptThatIsTurningInPlace.Heading;
import org.opencv.core.*;

import java.util.ArrayList;

public class ObjectProc {
    public static final double poleDiameter = 1; // poles are 1 inch or 25.4 mm in diameter and 33.5 inches or 851 mm tall

    public static final double camOffset=0;

    private static final org.firstinspires.ftc.teamcode.visionPipelines.pole noPole = new org.firstinspires.ftc.teamcode.visionPipelines.pole(new Size(),new Size(),0,0,0,new MatOfPoint(),false);

    static double largest;
    static double best;
    static double error;

    static org.firstinspires.ftc.teamcode.visionPipelines.pole bestPole;
    static double bestAngle;

    static ArrayList<MatOfPoint> toDraw = new ArrayList<MatOfPoint>();

    public static pole getPoleAt(ArrayList<pole> poles, Heading expectedHeading, double maxDeviation) {
        if (poles.size() > 0) {
            best = 69420;
            for (int i = 0; i < poles.size(); i++) {
                error = abs(poles.get(i).getAngle() - expectedHeading.asRR());
                if (error < best) {
                    best = error;
                    bestPole = poles.get(i);
                }
            }
            bestAngle = bestPole.getAngle();
            if (Math.abs((bestAngle-expectedHeading.asRR())/((bestAngle+expectedHeading.asRR())/2)) > maxDeviation) {
                bestPole=noPole;
            }
            else {
                toDraw.clear();
                toDraw.add(bestPole.contour);
                draw(toDraw);
            }

            return bestPole;
        }
        return noPole;
    }
    public static pole getCenterPole(ArrayList<pole> poles) {
        if (poles.size() > 0) {
            best = 69420;
            for (int i = 0; i < poles.size(); i++) {
                error = abs(poles.get(i).pos.width);
                if (error < best) {
                    best = error;
                    bestPole = poles.get(i);
                }
            }
            toDraw.clear();
            toDraw.add(bestPole.contour);
            draw(toDraw);
            return bestPole;
        }
        return noPole;
    }
    public static pole getBestPole(ArrayList<pole> poles) {
        if (poles.size() > 0) {
            largest = 0;
            for (int i = 0; i < poles.size(); i++) {
                if (poles.get(i).size.width > largest) {
                    largest = poles.get(i).size.width;
                    bestPole = poles.get(i);
                }
            }
            toDraw.clear();
            toDraw.add(bestPole.contour);
            draw(toDraw);
            return bestPole;
        }
        return noPole;
    }
}
