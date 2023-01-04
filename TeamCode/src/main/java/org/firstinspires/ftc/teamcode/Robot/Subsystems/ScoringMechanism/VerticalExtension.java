package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.MotionConstraint;
import org.firstinspires.ftc.teamcode.Utils.ProfiledPID;

public class VerticalExtension extends Subsystem {

	MainScoringMechanism.MechanismStates state = MainScoringMechanism.MechanismStates.BEGIN;

	DcMotorEx leftMotor;
	DcMotorEx rightMotor;

	PIDCoefficients slideCoefficients = new PIDCoefficients(0.1,0,0);
	MotionConstraint upConstraint = new MotionConstraint(10,10,10);
	MotionConstraint downConstraint = new MotionConstraint(10,10,10);

	ProfiledPID controller = new ProfiledPID(upConstraint,downConstraint,slideCoefficients);

	protected double slideTargetPosition = 0;

	public void commonInit(HardwareMap hwMap) {
		leftMotor = hwMap.get(DcMotorEx.class, "leftVertical");
		rightMotor = hwMap.get(DcMotorEx.class, "rightVertical");
		// TODO, set direction
		rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
	}
	@Override
	public void initAuto(HardwareMap hwMap) {
		commonInit(hwMap);
		leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}
	@Override
	public void initTeleop(HardwareMap hwMap) {
		commonInit(hwMap);
		leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	@Override
	public void periodic() {

		updatePID();
	}

	@Override
	public void shutdown() {

	}

	public void setState(MainScoringMechanism.MechanismStates state) {
		this.state = state;
		updateTargetPosition();
	}

	protected void updatePID() {
		double measuredPosition = getSlidePosition();
		double power = controller.calculate(slideTargetPosition,measuredPosition);
		leftMotor.setPower(power);
		rightMotor.setPower(power);
	}

	/**
	 * get position of the linear slides
	 * @return average encoder position of the slides
	 */
	public double getSlidePosition() {
		return (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2.0;
	}

	public double getSlideTargetPosition() {
		return slideTargetPosition;
	}

	public void updateTargetPosition() {
		switch (this.state) {
			case BEGIN:
				slideTargetPosition = 0.0;
				break;
			case LOW:
				// TODO: Find the correct position for this
				slideTargetPosition = 0.0;
				break;
			case MID:
				// TODO: Find the correct position for this
				slideTargetPosition = 0.0;
				break;
			case HIGH:
				// TODO: Find the correct position for this
				slideTargetPosition = 0.0;
				break;
			case GO_TO_LOW:
				// TODO: Find the correct position for this
				slideTargetPosition = 0.0;
				break;
			case TransferCone:
				// TODO: Find the correct position for this
				slideTargetPosition = 0.0;
				break;
		}
	}
}
