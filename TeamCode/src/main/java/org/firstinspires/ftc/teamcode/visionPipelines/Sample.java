package org.firstinspires.ftc.teamcode.visionPipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Sample {
    private Mat hsv = new Mat();
    private Mat lab = new Mat();
    private Mat ycrcb = new Mat();
    public Scalar avgRGB;
    public Scalar avgHSV;
    public Scalar avgLAB;
    public Scalar avgYCrCb;

    public Sample(Mat rgb) {
        Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV_FULL);
        Imgproc.cvtColor(rgb, lab, Imgproc.COLOR_RGB2Lab);
        Imgproc.cvtColor(rgb, ycrcb, Imgproc.COLOR_RGB2YCrCb);

        this.avgRGB = Core.mean(rgb);
        this.avgHSV = Core.mean(hsv);
        this.avgLAB = Core.mean(lab);
        this.avgYCrCb = Core.mean(ycrcb);
    }
}
