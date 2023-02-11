package org.firstinspires.ftc.teamcode.Math.Controllers.Coefficient;


/**
 * feedback controller that uses sqrt as a nonlinear way to counteract friction
 * <p>
 * is also coupled with a hysteresis
 * <p>
 * https://www.desmos.com/calculator/qiqimyikle
 */
public class SqrtCoefficients {


	public double K;
	public double Kd;
	public double H;

	/**
	 * Sqrt feedback controller
	 * <p>
	 * Take a look at: https://www.desmos.com/calculator/qiqimyikle before use
	 *
	 * @param K 'proportional' constant
	 * @param H hysteresis or minimum power
	 */
	public SqrtCoefficients(double K, double Kd, double H) {
		this.K = K;
		this.Kd = Kd;
		this.H = H;
	}


	public double getKd() {
		return Kd;
	}

	public double getH() {
		return H;
	}

	public double getK() {
		return K;
	}
}


