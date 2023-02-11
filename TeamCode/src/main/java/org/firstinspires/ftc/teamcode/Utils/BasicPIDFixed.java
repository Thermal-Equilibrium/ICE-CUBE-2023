package org.firstinspires.ftc.teamcode.Utils;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;

public class BasicPIDFixed extends BasicPID {
	public BasicPIDFixed(PIDCoefficients coefficients) {
		super(coefficients);
	}

	@Override
	public double getDT() {
		double dt = timer.currentTime();
		timer.reset();
		return dt;
	}

}
