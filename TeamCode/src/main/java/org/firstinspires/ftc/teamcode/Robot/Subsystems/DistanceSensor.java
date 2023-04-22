package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Utils.BallerFilter;
import org.firstinspires.ftc.teamcode.Utils.ExtraUtils;
import org.firstinspires.ftc.teamcode.Utils.LimitedSizeQueue;

public class DistanceSensor extends Subsystem {

    protected double distance_in = 0;
    //	KalmanFilter filter = new KalmanFilter(0.3,0.1,3);
    double constant = (1 / 0.003388888889);
    AnalogInput distance2;
    int QueueSize = 10;

    LimitedSizeQueue<Double> queue = new LimitedSizeQueue<>(QueueSize);

    // position of the pole assuming our distance sensor and odometry are perfect
    // since the robot might not actually be set to the real world coordinates its odometry is set to
    // or the distance sensor might have a constant bias amount, we want to work in relative terms.
    Pose2d polePositionInContext = new Pose2d();

    Drivetrain drivetrain;

    public DistanceSensor(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        // fill queue with zeros so we don't die.
        for (int i = 0; i < QueueSize; ++i) {
            queue.add(0.0);
        }
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        distance2 = hwMap.get(AnalogInput.class, "Distance2");
    }

    @Override
    public void periodic() {

        double sensor_volts = distance2.getVoltage();
        double sensor_inches = sensor_volts * constant;
        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.N) {
            BallerFilter filter = new BallerFilter(queue.toArray());
            distance_in = sensor_inches; //filter.computeResult();  //filter.estimate(sensor_inches);
            Dashboard.packet.put("DistanceKF", distance_in);
            Dashboard.packet.put("Sensor_Inches_raw", sensor_inches);
            ExtraUtils.drawPole(polePositionInContext, Dashboard.packet);
            queue.add(distance_in);
            System.out.println(queue);
        }

    }

    @Override
    public void shutdown() {

    }

    public double getDistance_in() {
        return distance_in;
    }

    public void calculatePolePositionInContext() {
        Pose2d robotPose = drivetrain.getPose();

        double distance = getDistance_in() + 4; // +4 is the approximate distance from the center of the robot.
        double Px = robotPose.getX() + Math.cos(robotPose.getHeading()) * distance;
        double Py = robotPose.getY() + Math.sin(robotPose.getHeading()) * distance;
        if (isValueBad()) {
            System.out.println("calculate context pole position failed");
            return;

        }
        polePositionInContext = new Pose2d(Px, Py, 0);
        System.out.println("calculate context pole position success at distance: " + distance);

    }

    public void backcalculateRobotPosition() {
        Pose2d robotPose = drivetrain.getPose();
        if (isValueBad()) {
            System.out.println("Back Calculate Failed");
            return;
        }
        double distance = getDistance_in() + 4; // +4 is the approximate distance from the center of the robot.
        double rX = polePositionInContext.getX() - Math.cos(robotPose.getHeading()) * distance;
        double rY = polePositionInContext.getY() - Math.sin(robotPose.getHeading()) * distance;
        drivetrain.setPose(new Pose2d(rX, rY, robotPose.getHeading()));
        System.out.println("Back Calculate Successful");

    }

    public boolean isValueBad() {
        return getDistance_in() > 15;
    }

    public Pose2d getPolePositionInContext() {
        return polePositionInContext;
    }
}
