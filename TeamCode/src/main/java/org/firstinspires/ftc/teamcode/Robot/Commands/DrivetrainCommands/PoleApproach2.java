package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.getFrameCount;
import static org.firstinspires.ftc.teamcode.visionPipelines.ObjectProc.getAngle;
import static org.firstinspires.ftc.teamcode.visionPipelines.ObjectProc.getBestPole;
import static org.firstinspires.ftc.teamcode.visionPipelines.ObjectProc.getDistance;
import static org.firstinspires.ftc.teamcode.visionPipelines.ObjectProc.getPoleAt;
import static org.firstinspires.ftc.teamcode.visionPipelines.ObjectProc.getWidthDegrees;
import static org.firstinspires.ftc.teamcode.visionPipelines.PolePipe.getPoles;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.ThermalEquilibrium.homeostasis.Utils.Vector;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Math.Controllers.MotorModelController;
import org.firstinspires.ftc.teamcode.Math.Controllers.TurnOnlyControl;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.visionPipelines.pole;
import org.opencv.core.MatOfPoint;

import java.util.ArrayList;

public class PoleApproach2 extends Command {
    static final double maxDistanceError = .25;

    static double power;

    boolean isComplete = false;
    static pole thePole;

    Drivetrain drivetrain;
    TurnOnlyControl angleController;
    MotorModelController distanceController;

    static double distance;
    static double currentTargetHeading;
    static double sum;
    static double widthDegrees;
    static double currentDistance;
    static ArrayList<Double> headingCache = new ArrayList<Double>();
    static ArrayList<Double> distanceCache = new ArrayList<Double>();
    static ElapsedTime timer = new ElapsedTime();
    static ElapsedTime timer2 = new ElapsedTime();
    static double visionUpdateTime = 100; // in ms
    static double currentVisionFrame;
    static double lastVisionFrame;

    static ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    static ArrayList<pole> poles = new ArrayList<pole>();

    static final double maxApproachError=Math.toRadians(4);
    static final double targetDistance=5; // in inches (kinda)

    static double expectedHeading;
    static boolean locked = false;
    static boolean excludeOutliers;

    static double posMaxDeviation =.5;

    static double meanMaxDeviation =.25;
    static double average;
    static double deviation;
    Vector test;
    @Config
    public static class distancePID {
        public static double kV = .0001;
        public static double kA = .0001;
        public static double kS = .0001;
        public static double maxAccel = 1;
    }

    static double mean(ArrayList<Double> numbers, boolean excludeOutliers){
        sum=0;
        for (int i=0; i<numbers.size(); i++){
            sum+=numbers.get(i);
        }
        average = sum / numbers.size();

        if (!excludeOutliers) { return average; }

        for (int i=0; i<numbers.size(); i++){
            deviation = (numbers.get(i) - average) / ((numbers.get(i) + average) / 2);
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
        power=distanceController.calculate(targetDistance, currentDistance, distancePID.maxAccel);
        Dashboard.packet.put("aaPower",power);
        drivetrain.robotRelative(new Pose2d(power,power));
//        drivetrain.setPower(power,power);
    }
    @RequiresApi(api = Build.VERSION_CODES.N)
    void face(){
        if (headingCache.size()!=0) {
            test = angleController.calculate();

//            drivetrain.setPower(test);
            drivetrain.robotRelative(new Pose2d(test.get(0),test.get(1)));
        }
        else { // no poles have been detected
            //drivetrain.setPower(0,0);
            drivetrain.robotRelative(new Pose2d(0,0));
        }
    }
    @RequiresApi(api = Build.VERSION_CODES.N)
    public PoleApproach2(Drivetrain drivetrain) {
        super(drivetrain);
        this.drivetrain = drivetrain;
        this.angleController = new TurnOnlyControl(() -> drivetrain.getPose().getHeading(),0);
        this.distanceController = new MotorModelController(distancePID.kV, distancePID.kA, distancePID.kS);
    }

    @Override
    public void init() {
        timer.reset();
        Vision.resumeView();
        lastVisionFrame=0;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void periodic() {
        if (timer.milliseconds()>= visionUpdateTime){
            headingCache.clear();
            distanceCache.clear();
            timer.reset();
        }
        currentVisionFrame=getFrameCount();
        if (currentVisionFrame>lastVisionFrame){ // make sure it isn't processing the same thing multiple times
            timer2.reset();
            poles = getPoles();
            if (locked) { thePole = getPoleAt(poles, expectedHeading, posMaxDeviation); }
            else {
                thePole=getBestPole(poles);
            }
            if (thePole.isReal) {
                locked = true;
                expectedHeading=Math.toRadians(getAngle(thePole)) + drivetrain.getPose().getHeading() + drivetrain.getVelocity().getHeading() * visionUpdateTime; // predict the heading of the pole in the next frame
            }
            else { locked = false; }
            if (locked) {
                widthDegrees=getWidthDegrees(thePole);
                distance=getDistance(widthDegrees);
                headingCache.add(Math.toRadians(getAngle(thePole)) + drivetrain.getPose().getHeading());
                distanceCache.add(distance);
                currentTargetHeading = mean(headingCache,true);
                currentDistance = mean(distanceCache,true);
                angleController.setHeadingReference(currentTargetHeading);
                Dashboard.packet.put("iwidth",thePole.size.width);
                Dashboard.packet.put("iheight",thePole.size.height);
                Dashboard.packet.put("idistance",distance);
                Dashboard.packet.put("Height",thePole.pos.height);
                Dashboard.packet.put("Ratio",thePole.ratio);
            }
            else {

//                drivetrain.setPower(0,0);
                drivetrain.robotRelative(new Pose2d(0,0));
            }
        }

        if (Math.abs(currentTargetHeading - drivetrain.getPose().getHeading())<=maxApproachError && headingCache.size() > 0) { approach(); }
        else { face(); }


        Dashboard.packet.put("time",timer2.milliseconds());
        Dashboard.packet.put("distance", currentDistance);
        Dashboard.packet.put("angle",currentTargetHeading);

        isComplete = Math.abs(angleController.getEndGoalError()) < Math.toRadians(2) && Math.abs(currentDistance - targetDistance) >= maxDistanceError && Math.abs(drivetrain.getVelocity().getHeading()) < Math.toRadians(10);
        lastVisionFrame=currentVisionFrame;
    }

    @Override
    public boolean completed() {
        if (isComplete){
            Vision.pauseView();
        }
        return isComplete;
//        return false;
    }

    @Override
    public void shutdown() {
//        drivetrain.setPower(0,0);
        drivetrain.robotRelative(new Pose2d(0,0));
    }


}
