package org.firstinspires.ftc.teamcode.Utils;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FeedbackController;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.MotionConstraint;

public class ProfiledPID implements FeedbackController {

	MotionConstraint verticalConstraint;
	MotionConstraint downConstraint;
	PIDCoefficients coefficients;
	BasicPID controller;


	public ProfiledPID(MotionConstraint verticalConstraint, MotionConstraint downConstraint,
					   PIDCoefficients coefficients) {
		this.verticalConstraint = verticalConstraint;
		this.downConstraint = downConstraint;
		this.coefficients = coefficients;
		this.controller = new BasicPID(coefficients);
	}

	ElapsedTime timer = new ElapsedTime();

	MotionProfile m_profile;

	double previousMotorTarget = 0;

	@Override
	public double calculate(double reference, double state) {
		generateMotionProfile(reference,state);
		double immediateReference = m_profile.get(timer.seconds()).getX();
		return controller.calculate(immediateReference,state);
	}

	protected void generateMotionProfile(double reference, double state) {
		if (reference != previousMotorTarget || m_profile == null) {
			timer.reset();

			if (reference > previousMotorTarget) {
				m_profile = MotionProfileGenerator.generateSimpleMotionProfile(
						new MotionState(state,0, 0),
						new MotionState(reference, 0, 0),
						verticalConstraint.max_velocity,
						verticalConstraint.max_acceleration,
						75
				);
			} else {
				m_profile = MotionProfileGenerator.generateSimpleMotionProfile(
						new MotionState(state, 0, 0),
						new MotionState(reference, 0, 0),
						downConstraint.max_velocity,
						downConstraint.max_acceleration,
						75
				);
			}

		}
		previousMotorTarget = reference;

	}
}
