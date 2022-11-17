package org.firstinspires.ftc.teamcode.visionPipelines;

public class Touching {
    boolean top;
    boolean bottom;
    boolean left;
    boolean right;
    boolean any;
    boolean vertical;
    boolean horizontal;
    public Touching(boolean top, boolean bottom, boolean left, boolean right) {
        this.top=top;
        this.bottom=bottom;
        this.left=left;
        this.right=right;
        if (top || bottom || left || right) { this.any=true; }
        else { this.any=false; }
        if (top || bottom) { this.vertical=true; }
        else { this.vertical=false; }
        if (left || right) { this.horizontal=true; }
        else { this.horizontal=false; }

    }
}
