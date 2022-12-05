package org.firstinspires.ftc.teamcode.visionPipelines;

import org.opencv.core.MatOfPoint;
import org.opencv.core.RotatedRect;
import org.opencv.core.Size;

public class MonocularPole {
    public Size pos;
    public Size size;
    public RotatedRect rect;
    public MatOfPoint contour;
    public Size dEstimate;

    public MonocularPole(Size pos, Size size, RotatedRect rect, MatOfPoint contour, Size dEstimate) {
        this.pos = pos;
        this.size = size;
        this.rect = rect;
        this.contour = contour;
        this.dEstimate = dEstimate;
    }
}