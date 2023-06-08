package org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.MainScoringMechanism;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.VerticalExtension;
@Config
public class VerticalTestingCommand extends Command {
    public static int position = 0;
    VerticalExtension verticalExtension;
    public VerticalTestingCommand(VerticalExtension verticalExtension) {
        this.verticalExtension = verticalExtension;
    }
    @Override
    public void init() {

    }

    @Override
    public void periodic() {
        switch (position) {
            case 0:
            case 1:
                verticalExtension.updateTargetPosition(VerticalExtension.IN_POSITION);
                return;
            case 2:
                verticalExtension.updateTargetPosition(VerticalExtension.MID_POSITION);
                return;
            case 3:
            case 8:
                verticalExtension.updateTargetPosition(VerticalExtension.HIGH_POSITION);
                return;
            case 4:
            case 9:
                verticalExtension.updateTargetPosition(VerticalExtension.ABOVE_POLE_POSITION);
                return;
        }

    }

    @Override
    public boolean completed() {
        return false;
    }

    @Override
    public void shutdown() {
        verticalExtension.shutdown();
    }
}
