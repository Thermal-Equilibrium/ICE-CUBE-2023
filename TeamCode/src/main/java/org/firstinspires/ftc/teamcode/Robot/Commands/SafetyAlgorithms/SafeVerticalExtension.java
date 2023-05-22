package org.firstinspires.ftc.teamcode.Robot.Commands.SafetyAlgorithms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.NullCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.RunCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.MoveVerticalExtension;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;

import java.util.function.Supplier;

@Config
public class SafeVerticalExtension extends Command {
    public static double upDangerLimit = 30;
    public static double downDangerLimit = 20;
    public static double upTimeCap = .9;
    public static double downTimeCap = .9;

    protected final VerticalExtension extension;
    private final double desiredPosition;
    private final ElapsedTime timer;

    private boolean stalling;
    private double danger;
    private double lastTime;
    private double initialPosition;

    public static Command SafeVerticalExtensionWithBackup (VerticalExtension verticalExtension, double desiredPosition) {
        SafeVerticalExtension mainExtension = new SafeVerticalExtension(verticalExtension, desiredPosition);
        Supplier<Command> runAfter = mainExtension::postProcedure;
        return mainExtension.addNext(new RunCommand(runAfter));
    }
    public SafeVerticalExtension(VerticalExtension extension, double desiredPosition) {
        this.extension = extension;
        this.desiredPosition = desiredPosition;
        this.timer = new ElapsedTime();
        this.stalling = false;
    }

    @Override
    public void init() {
        extension.updateTargetPosition(desiredPosition);
        this.timer.reset();
        this.lastTime = 0;
        this.danger = 0;
        this.initialPosition = this.extension.getSlidePosition();

    }

    @Override
    public void periodic() {
        this.stallCheck();
    }

    @Override
    public boolean completed() {
        return extension.isMovementFinished() || this.isDangerLimitExceeded() || this.isTimeCapExceeded();
    }

    @Override
    public void shutdown() {

    }


    private void stallCheck() {




        double targetDeviation = this.extension.getPIDTargetDeviation();

        double currentSpeedRatio = Math.abs(this.extension.getCurrent()) / this.extension.getSpeedInchesPerSecond();


        this.danger += currentSpeedRatio * targetDeviation * (this.timer.seconds() - this.lastTime);
        Dashboard.packet.put("DANGER", this.danger);
        this.lastTime = timer.seconds();
    }

    private double getDangerLimit() {
        if (desiredPosition - this.initialPosition > 0) {
            return upDangerLimit;
        }
        return downDangerLimit;
    }
    private boolean isDangerLimitExceeded() {
        return this.danger > this.getDangerLimit();
    }

    private double getTimeCap() {
        if (desiredPosition - this.initialPosition > 0) {
            return upTimeCap;
        }
        return downTimeCap;
    }
    private boolean isTimeCapExceeded() {
        return this.timer.seconds() > this.getTimeCap();
    }
    public Command postProcedure() {
        if (this.isTimeCapExceeded()) {
            Dashboard.packet.put("STALL PROTECTION", "time cap exceeded, movement canceled");
            return new MoveVerticalExtension(this.extension, this.initialPosition);
        }
        if (this.isDangerLimitExceeded()) {
            Dashboard.packet.put("STALL PROTECTION", "too much danger, movement canceled");
            return new MoveVerticalExtension(this.extension, this.initialPosition);
        }
        Dashboard.packet.put("STALL PROTECTION", "movement successful");
        return new NullCommand();
    }
}
