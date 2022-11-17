package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.dashboard;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.getCamWidth;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.getFov;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.getFrameCount;
import static org.firstinspires.ftc.teamcode.Utils.ExtraUtils.drawPole;
import static org.firstinspires.ftc.teamcode.visionPipelines.OdometryPipe.getPoles;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.visionPipelines.Pole;
import org.firstinspires.ftc.teamcode.visionPipelines.PoleVisual;

import java.util.ArrayList;

public class VisualOdometry extends Subsystem {
    /*
    WR = relative to webcam
    RR = relative to robot
    absolute = relative to field

    all angles are in radians
    all distances are in inches
     */

    static ArrayList<PoleVisual> rawPoles;
    Drivetrain drivetrain;
    double currentFrame;
    double lastFrame;
    double webcamFOV=getFov();
    double webcamWidth=getCamWidth();
    double poleDiameter=1;
    double webcamOffset=4; // webcam's distance from robot's pivot point

    double occupiedFOV;
    double distanceWR;
    double distanceWROLD;
    double distanceRR;
    double angleWR;
    double angleRR;
    double xRR;
    double yWR;
    double yRR;
    Pose2d robotPose;

    PoleVisual rawPole;
    Pole pole;

    public VisualOdometry(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    void process(ArrayList<PoleVisual> rawPoles) {
        for (int i = 0; i< rawPoles.size(); i++) {
            //use Math.exact functions??
            rawPole = rawPoles.get(i);
            angleWR = Math.toRadians( webcamFOV * rawPole.pos.width / webcamWidth );
            occupiedFOV = Math.toRadians( webcamFOV * rawPole.size.width / webcamWidth );
//            distanceWR = poleDiameter / ( Math.tan( .5 * occupiedFOV ) ) + poleDiameter / 2; // add .5 inch so that it is distance to the center of the pole, not the side the robot is facing
            distanceWR = poleDiameter / ( 2 * Math.sin( .5 * occupiedFOV ) ) + poleDiameter / 2; // old distance calculation method, should work when angle=0 but otherwise is not accurate
            xRR = distanceWR * Math.sin(angleWR);
//            yWR = distanceWR * Math.cos(angleWR);
            yWR = xRR / Math.tan(angleWR);
            yRR = yWR + webcamOffset;
            distanceRR = Math.sqrt( Math.pow(xRR, 2 ) + Math.pow( yRR , 2 ) );
            angleRR = Math.atan( xRR / yRR );
            robotPose = drivetrain.getPose();
            Vector2d vec = new Vector2d(robotPose.getY() + yRR,robotPose.getX() -xRR);
            vec = vec.rotated(robotPose.getHeading() + angleRR);
            pole = new Pole( new Pose2d(vec.getX(),vec.getY(), angleRR ) , rawPole.verticalAngle);//robotPose.getX() + //robotPose.getY() +//robotPose.getHeading() +
            drawPole(pole.pose,Dashboard.packet);
            Dashboard.packet.put("occupiedFOV", Math.toDegrees(occupiedFOV));
            Dashboard.packet.put("angleWR", Math.toDegrees(angleWR));
            Dashboard.packet.put("angleRR", Math.toDegrees(angleRR));
            Dashboard.packet.put("xRR", xRR);
            Dashboard.packet.put("yRR", yRR);
            Dashboard.packet.put("yWR", yWR);
            Dashboard.packet.put("distanceWR", distanceWR);
            Dashboard.packet.put("distanceRR", distanceRR);
        }

    }
    void run() {
        rawPoles = getPoles();
        process(rawPoles);
    }
    @Override
    public void initAuto(HardwareMap hwMap) {
        rawPoles = new ArrayList<PoleVisual>();
        lastFrame=0;

    }

    @Override
    public void periodic() {
        currentFrame=getFrameCount();
        if (currentFrame>lastFrame) { run(); }
        lastFrame=getFrameCount();
    }

    @Override
    public void shutdown() {

    }
}


