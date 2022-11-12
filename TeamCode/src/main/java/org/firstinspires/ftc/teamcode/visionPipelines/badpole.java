package org.firstinspires.ftc.teamcode.visionPipelines;


public class badpole {
	public double xPixel;
	public double width;
	public boolean isValidPole;

	public badpole() {
		this.isValidPole = false;
		this.xPixel = 0;
		this.width = 0;
	}

	public badpole(double xPixel, double width) {
		this.xPixel = xPixel;
		this.width = width;
		this.isValidPole = true;
	}

	@Override
	public String toString() {
		return "Pole{" +
				"xPixel=" + xPixel +
				", width=" + width +
				", isValidPole=" + isValidPole +
				'}';
	}
}
