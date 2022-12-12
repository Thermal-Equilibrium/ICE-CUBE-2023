package org.firstinspires.ftc.teamcode.CommandFramework;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Subsystem {

    ElapsedTime timer = new ElapsedTime();
    protected double deltaTime = 0;

    public abstract void initAuto(HardwareMap hwMap);

    public void initTeleop(HardwareMap hwMap) {
        initAuto(hwMap);
    }

    public abstract void periodic();

    public abstract void shutdown();

    public void startTimer() {
        timer.reset();
    }

    public void stopTimer() {
        deltaTime = timer.milliseconds();
        timer.reset();
    }

    public double getDelayLength() {
        return deltaTime;
    }
}
