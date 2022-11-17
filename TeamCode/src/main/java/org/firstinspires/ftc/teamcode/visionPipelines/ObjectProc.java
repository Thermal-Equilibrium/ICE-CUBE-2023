package org.firstinspires.ftc.teamcode.visionPipelines;

import static com.ThermalEquilibrium.homeostasis.Utils.MathUtils.normalizedHeadingError;
import static org.firstinspires.ftc.teamcode.visionPipelines.PolePipe.draw;

import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.Math.TheArcaneConceptThatIsTurningInPlace.Heading;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.opencv.core.*;

import java.util.ArrayList;

public class ObjectProc {
    public static final double poleDiameter = 1; // poles are 1 inch or 25.4 mm in diameter and 33.5 inches or 851 mm tall

    public static final double camOffset=0;

    private static final PoleVisual NO_RAW_POLE = new PoleVisual(new Size(),new Size(),0,0,0,new MatOfPoint(),false, new Touching(false,false,false,false));

    static double largest;
    static double best;
    static double error;

    static PoleVisual bestRawPole;
    static double bestAngle;

    static ArrayList<MatOfPoint> toDraw = new ArrayList<MatOfPoint>();

    public static PoleVisual getPoleAt(ArrayList<PoleVisual> rawPoles, Heading expectedHeading, double maxDeviation) {
        if (rawPoles.size() > 0) {
            best = 69420;
            for (int i = 0; i < rawPoles.size(); i++) {
                error = abs(rawPoles.get(i).getAngle() - expectedHeading.asRR());
                if (error < best) {
                    best = error;
                    bestRawPole = rawPoles.get(i);
                }
            }
            bestAngle = bestRawPole.getAngle();
            if (Math.abs((bestAngle-expectedHeading.asRR())/((bestAngle+expectedHeading.asRR())/2)) > maxDeviation) {
                bestRawPole = NO_RAW_POLE;
            }
            else {
                toDraw.clear();
                toDraw.add(bestRawPole.contour);
                draw(toDraw);
            }

            return bestRawPole;
        }
        return NO_RAW_POLE;
    }
    public static PoleVisual getPoleAt(ArrayList<PoleVisual> rawPoles, Heading expectedHeading, double maxDeviation, double currentHeading) {
        if (rawPoles.size() > 0) {
            best = 69420;
            for (int i = 0; i < rawPoles.size(); i++) {
                //error = abs(poles.get(i).getAngle() + currentHeading - expectedHeading.asFR());
                error = abs(normalizedHeadingError(expectedHeading.asFR(), rawPoles.get(i).getAngle() + currentHeading));
                if (error < best) {
                    best = error;
                    bestRawPole = rawPoles.get(i);
                }
            }
            bestAngle = bestRawPole.getAngle();
            Dashboard.packet.put("expected vs actual error", abs(normalizedHeadingError(expectedHeading.asFR(), bestRawPole.getAngle() + currentHeading)));
            if (abs(normalizedHeadingError(expectedHeading.asFR(), bestRawPole.getAngle() + currentHeading)) > maxDeviation) {//if (Math.abs((bestAngle-expectedHeading.asRR())/((bestAngle+expectedHeading.asRR())/2)) > maxDeviation) {
                bestRawPole = NO_RAW_POLE;
            }
            else {
                toDraw.clear();
                toDraw.add(bestRawPole.contour);
                draw(toDraw);
            }
            return bestRawPole;
        }
        return NO_RAW_POLE;
    }
    public static PoleVisual getCenterPole(ArrayList<PoleVisual> rawPoles) {
        if (rawPoles.size() > 0) {
            best = 69420;
            for (int i = 0; i < rawPoles.size(); i++) {
                error = abs(rawPoles.get(i).pos.width);
                if (error < best) {
                    best = error;
                    bestRawPole = rawPoles.get(i);
                }
            }
            toDraw.clear();
            toDraw.add(bestRawPole.contour);
            draw(toDraw);
            return bestRawPole;
        }
        return NO_RAW_POLE;
    }
    public static PoleVisual getBestPole(ArrayList<PoleVisual> rawPoles) {
        if (rawPoles.size() > 0) {
            largest = 0;
            for (int i = 0; i < rawPoles.size(); i++) {
                if (rawPoles.get(i).size.width > largest) {
                    largest = rawPoles.get(i).size.width;
                    bestRawPole = rawPoles.get(i);
                }
            }
            toDraw.clear();
            toDraw.add(bestRawPole.contour);
            draw(toDraw);
            return bestRawPole;
        }
        return NO_RAW_POLE;
    }
}
