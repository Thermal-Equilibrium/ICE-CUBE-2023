package org.firstinspires.ftc.teamcode.visionPipelines;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.FOV;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.getCamWidth;
import static org.firstinspires.ftc.teamcode.visionPipelines.PolePipe.draw;

import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

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

    public static double getAngle(org.firstinspires.ftc.teamcode.visionPipelines.pole thePole){
        return thePole.pos.width*(FOV/getCamWidth());
    }
    public static double getWidthDegrees(org.firstinspires.ftc.teamcode.visionPipelines.pole thePole){
        return ((thePole.pos.width/2) * (FOV/getCamWidth()))*2;
    }
    public static double getDistance(double widthDegrees){
        //double distance = poleDiameter/(2*Math.sin(Math.toRadians(.5*widthDegrees)));
        return poleDiameter/(Math.sin(Math.toRadians(.5*widthDegrees)));
    }
    public static pole getPoleAt(ArrayList<pole> poles, double expectedAngle, double maxDeviation) {
        if (poles.size() > 0) {
            best = 69420;
            for (int i = 0; i < poles.size(); i++) {
                error = abs(getAngle(poles.get(i)) - expectedAngle);
                if (error < best) {
                    best = error;
                    bestPole = poles.get(i);
                }
            }
            bestAngle = toRadians(getAngle(bestPole));
            if ((bestAngle-expectedAngle)/((bestAngle+expectedAngle)/2) > maxDeviation) {
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
