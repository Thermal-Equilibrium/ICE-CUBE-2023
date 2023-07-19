package org.firstinspires.ftc.teamcode.Purepursuit.Utils;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

/**
 * the realest filter, all of these individuals of suspicious backgrounds be toting cringe resources like CTRL ALT FTC
 * And about how the "kalman filter" is so amazing
 * <p>
 * ALl i can say about that is that no one has filters quite like
 * in-fact i have the best filters, filters unlike no other first tech challenge team.
 * <p>
 * Here is how i accomplished this amazing feat:
 * <p>
 * Both the median and the average filter do great at their own specific things.
 * <p>
 * Median filters are great at rejection obvious outliers while average filters are great at removing noise.
 * <p>
 * The two methods however do not do that well individually with their opposing skill-sets so i propose a W-rizz divide and conquer method.
 * <p>
 * The first stage is outlier removal, this takes the data set and removes ~40% of the data based on it's distance to the median.
 * <p>
 * Then we average tbe remaining values.  The resulting is a faster, more responsive filter since we are performing a tighter moving average than we otherwise would
 * <p>
 * While also being incredibly computationally efficient
 */
public class BallerFilter {

	Double[] data;
	double percentageToRemove = 0.5;
	int numToRemain;

	public BallerFilter(Double[] data) {
		this.data = data;
		this.numToRemain = (int) ((1 - percentageToRemove) * data.length);
	}


	public double computeResult() {
		Arrays.sort(data);
		double median;
		if (data.length % 2 == 0)
			median = (data[data.length / 2] + data[data.length / 2 - 1]) / 2;
		else
			median = data[data.length / 2];

		ArrayList<Double> data2 = new ArrayList<>();
		Collections.addAll(data2, data);


		while (data2.size() > numToRemain + 1) {
			double currentMaxDeviation = Math.abs(median - data2.get(0));
			double currentElementToRemove = data2.get(0);
			for (int i = 1; i < data2.size(); i++) {
				double currentDeviation = Math.abs(median - data2.get(0));
				if (currentDeviation > currentMaxDeviation) {
					currentMaxDeviation = currentDeviation;
					currentElementToRemove = data2.get(0);
				}
			}
			data2.remove(currentElementToRemove);
		}

		double sum = 0;
		for (double e :
				data2) {
			sum += e;
		}

		return sum / data.length;
	}


}
