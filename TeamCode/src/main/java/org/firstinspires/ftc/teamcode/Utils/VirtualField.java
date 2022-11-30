package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VirtualField {
    public enum PoleState {
        EMPTY,
        US,
        THEM,
    }

    static class Coordinate {
        int x;
        int y;
        boolean isEnabled;

        Coordinate(int x, int y, boolean isEnabled) {
            this.x = x;
            this.y = y;
            this.isEnabled = isEnabled;
        }

        public boolean isVisible(int x, int y) {
            return x == this.x && y == this.y && isEnabled;
        }
    }

    public enum PoleType {
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }

    PoleType[][] fieldMap = {
            {PoleType.GROUND,   PoleType.LOW,       PoleType.GROUND,    PoleType.LOW,       PoleType.GROUND},
            {PoleType.LOW,      PoleType.MEDIUM,    PoleType.HIGH,      PoleType.MEDIUM,    PoleType.LOW},
            {PoleType.GROUND,   PoleType.HIGH,      PoleType.GROUND,    PoleType.HIGH,      PoleType.GROUND},
            {PoleType.LOW,      PoleType.MEDIUM,    PoleType.HIGH,      PoleType.MEDIUM,    PoleType.LOW},
            {PoleType.GROUND,   PoleType.LOW,       PoleType.GROUND,    PoleType.LOW,       PoleType.GROUND},
    };

    public int GROUND_VALUE = 2;
    public int LOW_VALUE = 3;
    public int MEDIUM_VALUE = 4;
    public int HIGH_VALUE = 5;

    public int SUBSTATION_X = 4;
    public int SUBSTATION_Y = 2;

    public int SCORE_WEIGHT = 1;
    public int DISTANCE_WEIGHT = 1;


    public PoleState[][] field = new PoleState[5][5];

    Coordinate cursor = new Coordinate(0, 0, true);
    Coordinate target = new Coordinate(0, 0, false);

    Telemetry telemetry;



    public VirtualField(Telemetry telemetry) {
        // Initialize the field empty
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                field[i][j] = PoleState.EMPTY;
            }
        }
        this.telemetry = telemetry;
    }

    public void moveCursor(int xOffset, int yOffset) {
        cursor.x = (cursor.x + xOffset) % 5;
        cursor.y = (cursor.y + yOffset) % 5;
    }

    public void setPole(PoleState poleState) {
        field[cursor.x][cursor.y] = poleState;
    }

    public void draw() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        for (int y = 0; y < 5; y++) {
            StringBuilder line = new StringBuilder();
            for (int x = 0; x < 5; x++) {
                if (cursor.isVisible(x, y)) {
                    line.append( "ⵊ ");
                } else if (target.isVisible(x, y)) {
                    line.append( "╳ ");
                } else {
                    switch (field[x][y]) {
                        case EMPTY:
                            line.append( "· ");
                            break;
                        case US:
                            line.append( "◉ ");
                            break;
                        case THEM:
                            line.append( "◎ ");
                            break;
                    }
                }
            }
            telemetry.addLine(line.toString());
        }
    }

    public void runAlgorithm() {
        // Find the best target
        double bestScore = 0;
        for (int y = 0; y < 5; y++) {
            for (int x = 0; x < 5; x++) {
                double score = calculatePoleScore(x, y);
                if (score > bestScore) {
                    bestScore = score;
                    target.x = x;
                    target.y = y;
                }
            }
        }
        target.isEnabled = true;
    }

    public void runLoop() {
        runAlgorithm();
        draw();
    }

    public double calculatePoleScore(int x, int y) {
        PoleState currentState = field[x][y];
        PoleType currentType = fieldMap[x][y];
        int swing_score = 0;
        switch (currentType) {
            case GROUND:
                swing_score = GROUND_VALUE;
                break;
            case LOW:
                swing_score = LOW_VALUE;
                break;
            case MEDIUM:
                swing_score = MEDIUM_VALUE;
                break;
            case HIGH:
                swing_score = HIGH_VALUE;
                break;
        }

        if (currentState == PoleState.THEM) {
            swing_score += 6;
        } else if (currentState == PoleState.US) {
            swing_score += 3;
        }

        int distance_score = Math.abs(x - SUBSTATION_X) + Math.abs(y - SUBSTATION_Y);

        return (double) (swing_score * SCORE_WEIGHT) / (distance_score * DISTANCE_WEIGHT);
    }
}