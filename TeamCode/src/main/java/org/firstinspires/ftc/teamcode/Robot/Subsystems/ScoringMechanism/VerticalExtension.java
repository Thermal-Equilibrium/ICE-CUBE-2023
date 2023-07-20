package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

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
import org.firstinspires.ftc.teamcode.Purepursuit.Utils.ProfiledPID;

@Config
public class VerticalExtension extends Subsystem {

	public static double HIGH_POSITION = 28.5;
	public static double MID_POSITION = 20.5;
	public static double GROUND_POSITION = 2.0;
	public static double LOW_POSITION = 13.0;
	public static double CONE_5 = 5.25;
	public static double CONE_4 = 3.5;
	public static double CONE_3 = 2.0;
	public static double CONE_2 = 1.0;
	public static double CONE_1 = 0.0;


	public final static double IN_POSITION = 0.0;
	static final double PULLEY_CIRCUMFERENCE = 4.409;
	static final double counts_per_revolution = 145.090909;
	public static double Kp = 0.2;
	public static double Kd = 1.8 * Math.sqrt(Kp * 0.0015);
	public static double max_accel = 250.0;
	public static double max_velocity = 250.0;

	public static double DISTANCE_FOR_CONE = 8.5; // 9 inches or less means we still have the cone


	protected double slideTargetPosition = 0;
	protected double Kg = 0.09499;
	DcMotorEx vertical1;
	DcMotorEx vertical2;
	PIDCoefficients coefficients = new PIDCoefficients(Kp, 0, Kd);
	MotionConstraint upConstraint = new MotionConstraint(max_accel , max_accel / 2, max_velocity);
	MotionConstraint downConstraint = new MotionConstraint(max_accel / 1.5, max_accel / 2, max_velocity / 1.5);
	ProfiledPID controller = new ProfiledPID(upConstraint, downConstraint, coefficients);
	private VoltageSensor batteryVoltageSensor;
	protected double current = 0;

	public void commonInit(HardwareMap hwMap) {
		vertical1 = hwMap.get(DcMotorEx.class, "slidesRight");
		vertical2 = hwMap.get(DcMotorEx.class, "slidesLeft");
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
		upConstraint = new MotionConstraint(max_accel, max_accel / 2, max_velocity);
		downConstraint = new MotionConstraint(max_accel, max_accel / 2, max_velocity);

		vertical1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		vertical2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		vertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		vertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	@Override
	public void periodic() {

//		if (getSlidePosition() < 4 && currentLimitExceeded()) {
//			vertical1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			vertical2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			vertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//			vertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//		}

		Dashboard.packet.put(
				"slide height", getSlidePosition()
		);

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

		if (currentLimitExceeded()) {
			power /= 4;
		}

		vertical1.setPower(power);
		vertical2.setPower(power);
		Dashboard.packet.put("measured slide position", measuredPosition);
		Dashboard.packet.put("target slide position", controller.getTargetPosition());
		Dashboard.packet.put("slide power", power);
		current = vertical1.getCurrent(AMPS);
		Dashboard.packet.put("vertical current amps",current);
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

	public boolean currentLimitExceeded() {
		return Math.abs(current) > 4;
	}

	public double getCurrent() {
		return current;
	}


	public boolean slideIsDown() {
		return getSlidePosition() < 0.25;
	}

}
