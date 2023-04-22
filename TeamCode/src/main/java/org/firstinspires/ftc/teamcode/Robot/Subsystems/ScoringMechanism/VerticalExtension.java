package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.MotionConstraint;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Utils.ProfiledPID;

@Config
public class VerticalExtension extends Subsystem {

    public final static double HIGH_POSITION = 26.7;
    public final static double MID_POSITION = 16.3;
    public final static double MID_POSITION_teleop = 16.2;

    public final static double IN_POSITION = 0;
    static final double PULLEY_CIRCUMFERENCE = 4.409;
    static final double counts_per_revolution = 145.090909;
    public static double Kp = 0.2;
    public static double Kd = 1.3 * Math.sqrt(Kp * 0.0015);
    public static double max_accel = 190;
    public static double max_velocity = 200;
    protected double slideTargetPosition = 0;
    protected double Kg = 0.09499;
    MainScoringMechanism.MechanismStates state = MainScoringMechanism.MechanismStates.BEGIN;
    DcMotorEx vertical1;
    DcMotorEx vertical2;
    PIDCoefficients coefficients = new PIDCoefficients(Kp, 0, Kd);
    MotionConstraint upConstraint = new MotionConstraint(max_accel, max_accel, max_velocity);
    MotionConstraint downConstraint = new MotionConstraint(max_accel, max_accel, max_velocity);
    ProfiledPID controller = new ProfiledPID(upConstraint, downConstraint, coefficients);
    private VoltageSensor batteryVoltageSensor;

    public void commonInit(HardwareMap hwMap) {
        vertical1 = hwMap.get(DcMotorEx.class, "vertical1");
        vertical2 = hwMap.get(DcMotorEx.class, "vertical2");
        // TODO, set direction
        vertical2.setDirection(DcMotorSimple.Direction.REVERSE);
        vertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.batteryVoltageSensor = hwMap.voltageSensor.iterator().next();
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        commonInit(hwMap);
        vertical1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void initTeleop(HardwareMap hwMap) {
        commonInit(hwMap);
        vertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {

        updatePID();

    }

    @Override
    public void shutdown() {
        vertical1.setPower(0);
        vertical2.setPower(0);

    }

    protected void updatePID() {
        double measuredPosition = getSlidePosition();
        double power = controller.calculate(slideTargetPosition, measuredPosition);
        if (slideTargetPosition > IN_POSITION * 2) {
            power += Kg;
        } else {
            power -= Kg;
        }
        vertical1.setPower(power);
        vertical2.setPower(power);
        Dashboard.packet.put("measured slide position", measuredPosition);
        Dashboard.packet.put("target slide position", controller.getTargetPosition());
        Dashboard.packet.put("slide power", power);
    }

    /**
     * get position of the linear slides
     *
     * @return average encoder position of the slides
     */
    public double getSlidePosition() {
        return countsToInches((vertical1.getCurrentPosition() + vertical2.getCurrentPosition()) / 2.0);
    }

    public double getSlideTargetPosition() {
        return slideTargetPosition;
    }

    public void updateTargetPosition(double targetpos) {
        this.slideTargetPosition = targetpos;
    }

    public boolean isMovementFinished() {
        return controller.isDone();
    }

    public double countsToInches(double counts) {
        return (counts / counts_per_revolution) * PULLEY_CIRCUMFERENCE;
    }
}
