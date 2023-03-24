package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.Team;

@Autonomous
public class FastTeleAutoSingleMeasureBlue extends FastTeleAutoSingleMeasure{
    @Override
    public Team getTeam() {
        return Team.BLUE;
    }
}
