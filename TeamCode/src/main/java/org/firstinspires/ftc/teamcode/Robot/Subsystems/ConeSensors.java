package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Purepursuit.Utils.BallerFilter;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.NullCommand;

import java.util.ArrayList;

public class ConeSensors extends Subsystem {

	DistanceSensor sensorLeft;

	DistanceSensor sensorRight;
	final int sampleSize = 5;
	ArrayList<Double> leftData = new ArrayList<>();
	ArrayList<Double> rightData = new ArrayList<>();
	double leftFiltered = 100.0;
	double rightFiltered = 100.0;

	boolean previously_should_grab = false;

	Command c = new NullCommand();
	CommandScheduler scheduler;

	public ConeSensors(CommandScheduler scheduler) {
		this.scheduler = scheduler;
	}



	@Override
	public void initAuto(HardwareMap hwMap) {
		sensorLeft = hwMap.get(DistanceSensor.class, "colorLeft");
		sensorRight = hwMap.get(DistanceSensor.class, "colorRight");
	}

	@Override
	public void periodic() {
		double distLeft = sensorLeft.getDistance(DistanceUnit.CM);
		double distRight = sensorRight.getDistance(DistanceUnit.CM);

		leftData.add(distLeft);
		rightData.add(distRight);
		if(leftData.size() > sampleSize) {

			BallerFilter ballerFilter = new BallerFilter(leftData.toArray(new Double[0]));
			leftFiltered = ballerFilter.computeResult();
			leftData.clear();
		}

		if(rightData.size() > sampleSize) {
			BallerFilter ballerFilter = new BallerFilter(rightData.toArray(new Double[0]));
			rightFiltered = ballerFilter.computeResult();
			rightData.clear();
		}

		System.out.println("left filter: " + leftFiltered + " right filter: " + rightFiltered);

		boolean should_grab = leftFiltered <= 11.0 && rightFiltered <= 30.0;

		if(should_grab && !previously_should_grab) {
			this.scheduler.forceCommand(c);
		}

		previously_should_grab = should_grab;
	}

	public void setCommand(Command c) {
		this.c = c;
	}

	@Override
	public void shutdown() {

	}
}
