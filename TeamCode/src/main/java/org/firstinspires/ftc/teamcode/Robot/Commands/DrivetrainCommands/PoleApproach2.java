package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.getFrameCount;
import static org.firstinspires.ftc.teamcode.visionPipelines.ObjectProc.getBestPole;
import static org.firstinspires.ftc.teamcode.visionPipelines.ObjectProc.getPoleAt;
import static org.firstinspires.ftc.teamcode.visionPipelines.PolePipe.getPoles;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Math.TheArcaneConceptThatIsTurningInPlace.Heading;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DistanceSensor;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.returnToUltimateGoal.MotionProfiledADRCPoseStabilizationController;
import org.firstinspires.ftc.teamcode.visionPipelines.pole;
import org.opencv.core.MatOfPoint;

import java.util.ArrayList;

public class PoleApproach2 extends Command {

    static final double maxApproachError=Math.toRadians(4);
    public static double referenceDistanceSensor = 8.9;
    double error = 10;
    double error_tolerance = 1;

    public static PIDCoefficients turnCoefficients = new PIDCoefficients(.7,0,0.000000);//.000095
    public static PIDCoefficients controllerCoefficientsDistance = new PIDCoefficients(0.09,0,0.04);
    BasicPID controller;
    DistanceSensor distanceSensor;


    BasicPID turnController;
    AngleController turnControlWrapped;

    boolean isComplete = false;
    static pole thePole;

    Drivetrain drivetrain;

    static double distance;
    static Heading targetHeading;
    static Heading expectedHeading;


    static double currentDistance;

    static ArrayList<Double> headingCache = new ArrayList<Double>();
    static ArrayList<Double> distanceCache = new ArrayList<Double>();

    ElapsedTime timer = new ElapsedTime();
    static double clearCacheTimer = 1; // in ms

    double currentVisionFrame;
    double lastVisionFrame;

    static ArrayList<pole> poles = new ArrayList<pole>();

    boolean locked;

    static double posMaxDeviation =.5;
    static double meanMaxDeviation =.4;

    static double sum;
    static double average;
    static double deviation;

    @RequiresApi(api = Build.VERSION_CODES.N)
    public PoleApproach2(Drivetrain drivetrain,DistanceSensor distanceSensor) {
        super(drivetrain);
        this.drivetrain = drivetrain;
        this.distanceSensor = distanceSensor;
        this.locked = false;
        this.currentVisionFrame=0;
        this.lastVisionFrame=0;
        this.timer = new ElapsedTime();
        this.controller = new BasicPID(controllerCoefficientsDistance);
        this.turnController = new BasicPID(turnCoefficients);
        this.turnControlWrapped = new AngleController(turnController);
    }

    static double mean(ArrayList<Double> numbers, boolean excludeOutliers){
        sum=0;
        for (int i=0; i<numbers.size(); i++){
            sum+=numbers.get(i);
        }
        average = sum / numbers.size();

        if (!excludeOutliers) { return average; }

        for (int i=0; i<numbers.size(); i++){
            deviation = Math.abs((numbers.get(i) - average) / ((numbers.get(i) + average) / 2));
            if (deviation > meanMaxDeviation) { numbers.remove(i); }
        }
        numbers.add(average);
        sum=0;
        for (int i=0; i<numbers.size(); i++){
            sum+=numbers.get(i);
        }
        average = sum / numbers.size();

        return average;
    }
    void approach(){
        error = referenceDistanceSensor - distance;//distanceSensor.getDistance_in();
        double power = controller.calculate(referenceDistanceSensor, distanceSensor.getDistance_in()) + Math.signum(error) * 0.085;
        drivetrain.robotRelative(new Pose2d(-power,0,0));
    }
    @RequiresApi(api = Build.VERSION_CODES.N)
    void turn(){
        if (headingCache.size()!=0) {
            double turnPower = turnControlWrapped.calculate(0,targetHeading.asRR());
            drivetrain.robotRelative(new Pose2d(0,0,turnPower));
        }
        else { // no poles have been detected
            drivetrain.fieldRelative(new Pose2d(0,0,0));
        }
    }


    @Override
    public void init() {
        //Vision.resumeView();
        timer.reset();
        lastVisionFrame=0;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
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
            else {
                thePole=getBestPole(poles);
            }
            if (thePole.isReal) {
                locked = true;
                expectedHeading= new Heading(drivetrain.getPose(),thePole.getAngle() + drivetrain.getVelocity().getHeading() * .02, true); // predict the heading of the pole in the next frame
            }
            else { locked = false; }
            if (locked) {
                distance=thePole.getDistance();
                headingCache.add(thePole.getAngle());
                distanceCache.add(distance);
                targetHeading = new Heading(drivetrain.getPose(),mean(headingCache,true),true);
                currentDistance = mean(distanceCache,true);

                Dashboard.packet.put("idistance",distance);
//                Dashboard.packet.put("Height",thePole.pos.height);
//                Dashboard.packet.put("Ratio",thePole.ratio);

            }
            else {
                drivetrain.robotRelative(new Pose2d(0,0,0));
            }
        }
        if (headingCache.size() > 0) {
            if (Math.abs(targetHeading.asRR())<=maxApproachError && headingCache.size() > 0 && !(distanceSensor.getDistance_in() > 100)) { approach(); }
            else { turn(); }

        }

//        Dashboard.packet.put("raw x",);

        Dashboard.packet.put("Heading Speed",drivetrain.getVelocity().getHeading());
        Dashboard.packet.put("Current Heading (FR)",drivetrain.getPose().getHeading());

        Dashboard.packet.put("Target Heading (FR)",targetHeading.asFR());
        Dashboard.packet.put("Target Heading (RR)",targetHeading.asRR());

        Dashboard.packet.put("Predicted Heading (RR)",expectedHeading.asRR());
        Dashboard.packet.put("Predicted Heading (FR)",expectedHeading.asFR());

        Dashboard.packet.put("Complete Heading",Math.abs(targetHeading.asRR()) - maxApproachError);
        Dashboard.packet.put("Complete Velocity Heading",Math.abs(drivetrain.getVelocity().getHeading()) - Math.toRadians(6));
        Dashboard.packet.put("Complete Distance",Math.abs(error) - error_tolerance);

        Dashboard.packet.put("pos",thePole.pos.width);


        isComplete = Math.abs(targetHeading.asRR()) < maxApproachError && Math.abs(drivetrain.getVelocity().getHeading()) < Math.toRadians(6) && headingCache.size() > 0 && Math.abs(error) < error_tolerance;
        lastVisionFrame=currentVisionFrame;
    }

    @Override
    public boolean completed() {
        if (isComplete){
//            Vision.pauseView();
            drivetrain.robotRelative(new Pose2d(0,0,0));
        }
        return isComplete;
//        return false;
    }

    @Override
    public void shutdown() {
        drivetrain.shutdown();
    }


}
