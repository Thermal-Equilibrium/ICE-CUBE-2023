package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.MotionConstraint;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Utils.ProfiledPID;

public class VerticalExtension extends Subsystem {

	MainScoringMechanism.MechanismStates state = MainScoringMechanism.MechanismStates.BEGIN;

	DcMotorEx vertical1;
	DcMotorEx vertical2;

	PIDCoefficients coefficients = new PIDCoefficients(0.005,0,1.3 * Math.sqrt(0.005));
	MotionConstraint upConstraint = new MotionConstraint(3500,3500,2000);
	MotionConstraint downConstraint = new MotionConstraint(3500,10000,3000);

	ProfiledPID controller = new ProfiledPID(upConstraint,downConstraint,coefficients);
	public final static double HIGH_POSITION = 867;
	public final static double MID_POSITION = 545;

	public final static double IN_POSITION = 5;


	protected double slideTargetPosition = 0;

	protected double Kg = 0.09499;


	public void commonInit(HardwareMap hwMap) {
		vertical1 = hwMap.get(DcMotorEx.class, "vertical1");
		vertical2 = hwMap.get(DcMotorEx.class, "vertical2");
		// TODO, set direction
		vertical2.setDirection(DcMotorSimple.Direction.REVERSE);
		vertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		vertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

	}

	protected void updatePID() {
		double measuredPosition = getSlidePosition();
		double power = controller.calculateNoMotionProfile(slideTargetPosition,measuredPosition) + Kg;
		if (slideTargetPosition == MID_POSITION) {
			power = Range.clip(power,-1,1);
			power *= 0.8;
		}
		vertical1.setPower(power);
		vertical2.setPower(power);
		Dashboard.packet.put("measured slide position",measuredPosition);
		Dashboard.packet.put("target slide position",slideTargetPosition);
		Dashboard.packet.put("slide power",power);
	}

	/**
	 * get position of the linear slides
	 * @return average encoder position of the slides
	 */
	public double getSlidePosition() {
		return (vertical1.getCurrentPosition() + vertical2.getCurrentPosition()) / 2.0;
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
}
