package org.firstinspires.ftc.teamcode.Math.Controllers;


import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FeedbackController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * a position control class that also utilizes velocity feedforward / feedback
 */
public class GoodPositionControl implements FeedbackController {

	protected double state_previous = 0;
	PIDEx positionPID;
	PIDEx velocityPID;
	BasicFeedforward feedforward;
	ElapsedTime timer = new ElapsedTime();
	KalmanFilter filter = new KalmanFilter(0.3, 0.1, 3);

	public GoodPositionControl(PIDCoefficientsEx positionCoefficients,
							   PIDCoefficientsEx velocityCoefficients,
							   FeedforwardCoefficients feedforwardCoefficients) {
		this.positionPID = new PIDEx(positionCoefficients);
		this.velocityPID = new PIDEx(velocityCoefficients);
		this.feedforward = new BasicFeedforward(feedforwardCoefficients);
	}

	/**
	 * @param reference - target position
	 * @param state     - current position
	 * @return motor power to go to that position
	 */
	@Override
	public double calculate(double reference, double state) {
		double targetVelocity = positionPID.calculate(reference, state);
		double estimatedVelocity = (state - state_previous) / timer.seconds();
		estimatedVelocity = filter.estimate(estimatedVelocity);
		return velocityPID.calculate(targetVelocity, estimatedVelocity) + feedforward.calculate(state, targetVelocity, 0);
	}
}
