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
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.returnToUltimateGoal.MotionProfiledADRCPoseStabilizationController;
import org.firstinspires.ftc.teamcode.visionPipelines.pole;
import org.opencv.core.MatOfPoint;

import java.util.ArrayList;

public class PoleApproach2 extends Command {
    BasicPID turnController;
    AngleController turnControlWrapped;

    boolean isComplete = false;
    static pole thePole;

    Drivetrain drivetrain;

    MotionProfiledADRCPoseStabilizationController wtf;

    static double distance;
    static Heading targetHeading;
    static Heading expectedHeading;


    static double currentDistance;

    static ArrayList<Double> headingCache = new ArrayList<Double>();
    static ArrayList<Double> distanceCache = new ArrayList<Double>();

    static ElapsedTime timer = new ElapsedTime();
    static double clearCacheTimer = 100; // in ms

    static double currentVisionFrame;
    static double lastVisionFrame;

    static ArrayList<pole> poles = new ArrayList<pole>();

    static boolean locked = false;

    static final double maxDistanceError = .25;
    static double posMaxDeviation =.5;
    static double meanMaxDeviation =.25;

    static final double maxApproachError=Math.toRadians(4);
    static final double targetDistance=5; // in inches (kinda)

    static boolean excludeOutliers;
    static double sum;
    static double average;
    static double deviation;
    //@Config
    public static class distancePID {
        public static double kV = .0001;
        public static double kA = .0001;
        public static double kS = .0001;
        public static double maxAccel = 1;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public PoleApproach2(Drivetrain drivetrain) {
        super(drivetrain);
        this.drivetrain = drivetrain;

        turnController = new BasicPID(new PIDCoefficients(0.4,.1,0));
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
//    void approach(){
//        power=distanceController.calculate(targetDistance, currentDistance, distancePID.maxAccel);
////        drivetrain.robotRelative(new Pose2d(power,power));
//    }
    @RequiresApi(api = Build.VERSION_CODES.N)
    void turn(){
        if (headingCache.size()!=0) {
//            drivetrain.fieldRelative(wtf.goToPosition(new Pose2d(drivetrain.getPose().getX(),drivetrain.getPose().getY(),targetHeading.asFR()), drivetrain.getPose()));
            //drivetrain.robotRelative(wtf.goToPosition(new Pose2d(0,0,targetHeading.asFR()), new Pose2d(0,0,0)));
            double turnPower = turnControlWrapped.calculate(targetHeading.asFR(),drivetrain.getPose().getHeading());
            turnPower = Range.clip(turnPower,-0.3,0.3);
            drivetrain.robotRelative(new Pose2d(0,0,turnPower));

//            Pose2d powers = new Pose2d(0,0,targetHeading.asRR());
//            drivetrain.robotRelative(powers);
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

//                Dashboard.packet.put("iwidth",thePole.size.width);
//                Dashboard.packet.put("iheight",thePole.size.height);
//                Dashboard.packet.put("idistance",distance);
//                Dashboard.packet.put("Height",thePole.pos.height);
//                Dashboard.packet.put("Ratio",thePole.ratio);

            }
            else {
//                drivetrain.fieldRelative(new Pose2d(0,0,0));
                drivetrain.robotRelative(new Pose2d(0,0,0));
            }
        }

//        if (Math.abs(currentTargetHeading - drivetrain.getPose().getHeading())<=maxApproachError && headingCache.size() > 0) { approach(); }
//        else { turn(); }
        turn();

        Dashboard.packet.put("Heading Speed",drivetrain.getVelocity().getHeading());
        Dashboard.packet.put("Current Heading (FR)",drivetrain.getPose().getHeading());

        Dashboard.packet.put("Target Heading (FR)",targetHeading.asFR());
        Dashboard.packet.put("Target Heading (RR)",targetHeading.asRR());

        Dashboard.packet.put("Predicted Heading (RR)",expectedHeading.asRR());
        Dashboard.packet.put("Predicted Heading (FR)",expectedHeading.asFR());

        Dashboard.packet.put("pos",thePole.pos.width);


        //isComplete = Math.abs(angleController.getEndGoalError()) < Math.toRadians(2) && Math.abs(currentDistance - targetDistance) >= maxDistanceError && Math.abs(drivetrain.getVelocity().getHeading()) < Math.toRadians(10);
        lastVisionFrame=currentVisionFrame;
    }

    @Override
    public boolean completed() {
//        if (isComplete){
//            Vision.pauseView();
//        }
//        return isComplete;
        return false;
    }

    @Override
    public void shutdown() {
        drivetrain.shutdown();
    }


}
