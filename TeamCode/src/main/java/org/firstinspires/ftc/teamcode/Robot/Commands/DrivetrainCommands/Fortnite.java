package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

//import static org.firstinspires.ftc.teamcode.visionPipelines.ObjectProc.getBestPole;
//import static org.firstinspires.ftc.teamcode.visionPipelines.ObjectProc.getPoleAt;
//import static org.firstinspires.ftc.teamcode.visionPipelines.PolePipe.getPoles;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Math.TheArcaneConceptThatIsTurningInPlace.Heading;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.visionPipelines.MonocularPole;

import java.util.ArrayList;

public class Fortnite extends Command  {
    Drivetrain drivetrain;
    BasicPID turnControllerUnwrapped;
    BasicPID distanceController;
    AngleController turnController;
    KalmanFilter distanceFilter;

    public static PIDCoefficients turnCoefficients = new PIDCoefficients(1, .000000, 0.00000);//.7,.0000085,0.00000
    public static PIDCoefficients distanceCoefficients = new PIDCoefficients(0.09, 0, 0.04);

    static final double maxApproachError=Math.toRadians(3);
    static final double maxFinalTurnError=Math.toRadians(3);
    static final double maxFinalDistanceError = .4;

    ArrayList<MonocularPole> rawPoles = new ArrayList<MonocularPole>();
    MonocularPole targetRawPole;

    static double posMaxDeviation =.5;
    static double turningFinishedError=Math.toRadians(1.5);

    double distance;
    double distanceRaw;
    static double targetDistance = 7.9;
    Heading poleHeading;

    boolean hasPole;
    boolean doneTurning;

    boolean firstLoop;
    boolean isVictoryRoyale;

    double error;
    double turnPower;


    boolean turn(){
        error = poleHeading.asFR()-drivetrain.getPose().getHeading();
        turnPower = turnController.calculate(0,error);
        Dashboard.packet.put("Field Heading", drivetrain.getPose().getHeading());
        Dashboard.packet.put("Target Heading", poleHeading.asFR());
        Dashboard.packet.put("Turn Power", turnPower);
        Dashboard.packet.put("the Error", error);
        drivetrain.robotRelative(new Pose2d(0,0,turnPower));
        if (Math.abs(error) <= turningFinishedError) {
            return true;
        }
        else {
            return false;
        }

    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public Fortnite(Drivetrain drivetrain) {
        super(drivetrain);
        this.drivetrain = drivetrain;

        this.distanceController = new BasicPID(distanceCoefficients);
        this.distanceFilter = new KalmanFilter(0.3,0.1,3);
        this.turnControllerUnwrapped = new BasicPID(turnCoefficients);
        this.turnController = new AngleController(turnControllerUnwrapped);

        this.firstLoop=true;
        this.hasPole=false;
        isVictoryRoyale=false;

    }

    @Override
    public void init() { }

    @Override
    public void periodic() {
//        if (hasPole) {
//            doneTurning=turn();
//            if (doneTurning) {
//                hasPole=false;
//                drivetrain.robotRelative(new Pose2d(0,0,0));
//            }
//        }
//        else {
//            drivetrain.robotRelative(new Pose2d(0,0,0));
//            rawPoles = getPoles();
//            if (firstLoop){
//                targetRawPole =getBestPole(rawPoles);
//            }
//            else {
//                targetRawPole =getPoleAt(rawPoles,poleHeading,posMaxDeviation,drivetrain.getPose().getHeading());
//            }
////            if (targetRawPole.isReal) {
//            if (true) {
//                poleHeading=null;//new Heading(drivetrain.getPose(), targetRawPole.angle,true);
//                hasPole=true;
//                firstLoop=false;
//            }
//
//        }
//        Dashboard.packet.put("1First Loop", firstLoop);
//        Dashboard.packet.put("1Has Pole", hasPole);
//        Dashboard.packet.put("1First Loop", firstLoop);
//        Dashboard.packet.put("1Done Turning", doneTurning);
    }

    @Override
    public boolean completed() {
        if (isVictoryRoyale){
            drivetrain.robotRelative(new Pose2d(0,0,0));
        }
        return false;
    }

    @Override
    public void shutdown() {
        drivetrain.shutdown();
    }









}
