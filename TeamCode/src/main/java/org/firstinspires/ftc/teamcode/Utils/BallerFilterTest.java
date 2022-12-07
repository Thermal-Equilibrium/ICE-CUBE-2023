package org.firstinspires.ftc.teamcode.Utils;

public class BallerFilterTest {

	public static void main(String[] args) {
		Double[] data = {1.0,20.0,0.5,1.1,0.9,-123.0,3.0,0.0,-1.01};
		BallerFilter filter = new BallerFilter(data);
		System.out.println("filter result: " + filter.computeResult());
		System.out.println(filter.numToRemain);
	}
}
