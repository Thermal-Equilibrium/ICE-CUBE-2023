package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.getFrameCount;
import static org.firstinspires.ftc.teamcode.visionPipelines.ObjectProc.getBestPole;
import static org.firstinspires.ftc.teamcode.visionPipelines.ObjectProc.getPoleAt;
import static org.firstinspires.ftc.teamcode.visionPipelines.PolePipe.getPoles;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Math.TheArcaneConceptThatIsTurningInPlace.Heading;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DistanceSensor;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.visionPipelines.pole;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class PoleApproach2 extends Command {
    static double loseLockFrames = 7;

    static final double maxApproachError=Math.toRadians(1.5);
    static final double maxFinalTurnError=Math.toRadians(1.5);
    public static double referenceDistanceSensor = 7.9;
    double distanceError = 10;
    double error_tolerance = .4;

    public static PIDCoefficients turnCoefficients = new PIDCoefficients(.7,.0000085,0.00000);//
    public static PIDCoefficients distanceCoefficients = new PIDCoefficients(0.09,0,0.04);
    BasicPID distanceController;
    DistanceSensor distanceSensor;


    BasicPID turnController;
    AngleController turnControlWrapped;

    boolean isComplete = false;
    pole thePole;

    Drivetrain drivetrain;

     double distance;
     double distanceRaw;
     Heading targetHeading;
     Heading expectedHeading;

     double currentDistance;

     ArrayList<Double> headingCache = new ArrayList<Double>();
     ArrayList<Double> distanceCache = new ArrayList<Double>();

    ElapsedTime timer = new ElapsedTime();
    double clearCacheTimer = 1; // in ms

    double currentVisionFrame;
    double lastVisionFrame;

    ArrayList<pole> poles = new ArrayList<pole>();

    boolean locked;

    double posMaxDeviation =.5;
    double meanMaxDeviation =.4;

    double sum;
    double average;
    double deviation;

    double framesSincePole;

    KalmanFilter distanceFilter;

    BooleanSupplier exitSupplier;

    @RequiresApi(api = Build.VERSION_CODES.N)
    public PoleApproach2(Drivetrain drivetrain,DistanceSensor distanceSensor) {
        super(drivetrain);
        this.drivetrain = drivetrain;
        this.distanceSensor = distanceSensor;
        this.locked = false;
        this.currentVisionFrame=0;
        this.lastVisionFrame=0;
        this.timer = new ElapsedTime();
        this.distanceController = new BasicPID(distanceCoefficients);
        this.turnController = new BasicPID(turnCoefficients);
        this.turnControlWrapped = new AngleController(turnController);
        this.distanceFilter = new KalmanFilter(0.3,0.1,3);
        this.framesSincePole = 0;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public PoleApproach2(Drivetrain drivetrain,DistanceSensor distanceSensor, BooleanSupplier exitSupplier) {
        super(drivetrain);
        this.drivetrain = drivetrain;
        this.distanceSensor = distanceSensor;
        this.locked = false;
        this.currentVisionFrame=0;
        this.lastVisionFrame=0;
        this.timer = new ElapsedTime();
        this.distanceController = new BasicPID(distanceCoefficients);
        this.turnController = new BasicPID(turnCoefficients);
        this.turnControlWrapped = new AngleController(turnController);
        this.distanceFilter = new KalmanFilter(0.3,0.1,3);
        this.framesSincePole = 0;
        this.exitSupplier = exitSupplier;
    }

    void approach(){
        distanceError = referenceDistanceSensor - distance;//distanceSensor.getDistance_in();
        double power = distanceController.calculate(referenceDistanceSensor, distance) + Math.signum(distanceError) * 0.025;//distanceSensor.getDistance_in()
        Dashboard.packet.put("power.",power);
        power = Range.clip(power,-.30,.30);
        drivetrain.robotRelative(new Pose2d(-power,0,0));
    }

    void turn(){
        double turnPower = turnControlWrapped.calculate(0,targetHeading.asRR());
        turnPower = Range.clip(turnPower,-.20,.20);
        drivetrain.robotRelative(new Pose2d(0,0,turnPower));
    }
    void pixelTurn(){
        double turnPower = turnControlWrapped.calculate(0,thePole.pos.width);
        turnPower = -1*Range.clip(turnPower,-.11,.11);
        drivetrain.robotRelative(new Pose2d(0,0,turnPower));
    }
    @Override
    public void init() {
        timer.reset();
        lastVisionFrame=0;
    }

    @Override
    public void periodic() {
        if (timer.milliseconds()>= clearCacheTimer){
            headingCache.clear();
            distanceCache.clear();
            timer.reset();
        }
        currentVisionFrame=getFrameCount();
        if (currentVisionFrame>lastVisionFrame){ // make sure it isn't processing the same thing multiple times

            poles = getPoles();

            if (locked) { thePole = getPoleAt(poles, expectedHeading, posMaxDeviation); }
            else { thePole=getBestPole(poles); }

            if (thePole.isReal) { locked = true; framesSincePole=0; }
            else { framesSincePole++; }

            if (framesSincePole >= loseLockFrames) { locked = false; }

            if (thePole.isReal) {

                targetHeading = new Heading(drivetrain.getPose(), thePole.angle, true);

                expectedHeading = new Heading(drivetrain.getPose(), targetHeading.asRR() - drivetrain.getVelocity().getHeading() * .02, true); // predict the heading of the pole in the next frame

                distanceRaw = thePole.distance;
                distance = distanceFilter.estimate(distanceRaw);
                currentDistance = distance;

                Dashboard.packet.put("distanceRaw",distanceRaw);
                Dashboard.packet.put("distance",distance);
                Dashboard.packet.put("Ratio",thePole.ratio);
                Dashboard.packet.put("posX",thePole.pos.width);
                Dashboard.packet.put("posY",thePole.pos.height);
                Dashboard.packet.put("Target Heading (RR)",targetHeading.asRR());
                Dashboard.packet.put("Predicted Heading (RR)",expectedHeading.asRR());
                Dashboard.packet.put("Complete (Heading)",Math.abs(targetHeading.asRR()) - maxApproachError);
                Dashboard.packet.put("Complete (Distance)",Math.abs(distanceError) - error_tolerance);
                Dashboard.packet.put("Current Heading error (deg)",Math.toDegrees(targetHeading.asRR()));


                if (Math.abs(targetHeading.asRR()) < maxApproachError && currentDistance < 40) { approach(); } //!(distanceSensor.getDistance_in() > 100)
                else {
                    if (Math.abs(targetHeading.asRR())<.1) { pixelTurn(); }
                    else { turn(); }
                }

                isComplete = Math.abs(targetHeading.asRR()) < maxFinalTurnError && Math.abs(drivetrain.getVelocity().getHeading()) < Math.toRadians(3) && Math.abs(distanceError) < error_tolerance;
            }
            else {
                if (locked) {
                    expectedHeading = new Heading(drivetrain.getPose(),expectedHeading.asFR() - drivetrain.getVelocity().getHeading() * .02,false);
//                    drivetrain.fieldRelative(new Pose2d(0,0,expectedHeading.asFR()));

                    Dashboard.packet.put("Predicted Heading (RR)",expectedHeading.asRR());
                    Dashboard.packet.put("Predicted Heading (FR)",expectedHeading.asRR());
                    drivetrain.robotRelative(new Pose2d(0,0,0));
                }
                else {
                    drivetrain.robotRelative(new Pose2d(0,0,0));

                }



            }
        }

        Dashboard.packet.put("Locked",locked);
        if (thePole != null) {
            Dashboard.packet.put("Isreal?", thePole.isReal);
        }

        Dashboard.packet.put("Heading Speed",drivetrain.getVelocity().getHeading());
        Dashboard.packet.put("Complete (Velocity Heading)",Math.abs(drivetrain.getVelocity().getHeading()) - Math.toRadians(6));

        lastVisionFrame=currentVisionFrame;
    }

    @Override
    public boolean completed() {
        if (isComplete){
            drivetrain.robotRelative(new Pose2d(0,0,0));
        }

        boolean gamepadExit = true;

        if (exitSupplier != null) {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
                gamepadExit = exitSupplier.getAsBoolean();
            }
        }

        return isComplete || !gamepadExit;
//        return false;
    }

    @Override
    public void shutdown() {
        drivetrain.shutdown();
    }


}
