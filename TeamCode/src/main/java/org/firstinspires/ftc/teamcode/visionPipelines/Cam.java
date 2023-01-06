package org.firstinspires.ftc.teamcode.visionPipelines;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

import java.util.ArrayList;

public class Cam {
    Size nativeRes;
    double focalLen;// = 1; //mm
    double fx;// top left
    double fy;// middle middle
    double mx;// in px/mm
    double my;// in px/mm
    double m;

    double camID;
//    public Size position;
    public Pose2d position;

    Mat camMat;
    Mat newCamMat;
    Mat dists;
    public Size res;
    public double FOV;

    Mat theOut = new Mat();

    public ArrayList<MonocularPole> detectedPoles;
    public double currentFrame;
    public double lastFrame;

    public Cam(double camID, Size res, Pose2d position, double FOV, Size nativeRes, double focalLen) {
        this.camID = camID;
        this.res = res;
        this.position=position;
        this.FOV=FOV;

        this.nativeRes = nativeRes;
        this.focalLen = focalLen;

        this.camMat = this.loadCamMatrix();
        this.newCamMat = this.loadNewCamMatrix();
        this.dists = this.loadDists();

        this.fx = this.camMat.get(0,0)[0];
        this.fy = this.camMat.get(1,1)[0];
        this.mx = this.fx / this.focalLen;
        this.my = this.fy / this.focalLen;
        this.m = (this.mx + this.my) / 2;

        this.detectedPoles = new ArrayList<MonocularPole>();
        this.currentFrame=0;
        this.lastFrame=0;
    }

    private Mat loadCamMatrix() {
        Mat camMat = new Mat(3, 3, CvType.CV_64F, new Scalar(0));
        if ( this.camID == 0 ) {
            camMat.put(0, 0, 1426.5954310779582);
            camMat.put(0, 1, 0.0);
            camMat.put(0, 2, 976.06694339774);

            camMat.put(1, 0, 0.0);
            camMat.put(1, 1, 1427.6090838901566);
            camMat.put(1, 2, 523.6314205483386);

            camMat.put(2, 0, 0.0);
            camMat.put(2, 1, 0.0);
            camMat.put(2, 2, 1.0);
        }
        else if( this.camID == 1 ) {
            camMat.put(0, 0, 1426.5954310779582);
            camMat.put(0, 1, 0.0);
            camMat.put(0, 2, 976.06694339774);

            camMat.put(1, 0, 0.0);
            camMat.put(1, 1, 1427.6090838901566);
            camMat.put(1, 2, 523.6314205483386);

            camMat.put(2, 0, 0.0);
            camMat.put(2, 1, 0.0);
            camMat.put(2, 2, 1.0);
        }

        // ^ getting these values is way easier in python

        return camMat;
    }

    private Mat loadNewCamMatrix() {
        Mat newCamMat = new Mat(3, 3, CvType.CV_64F, new Scalar(0));
        if ( this.camID == 0 ) {
            newCamMat.put(0, 0, 1417.0025634765625);
            newCamMat.put(0, 1, 0.0);
            newCamMat.put(0, 2, 974.1928097073833);

            newCamMat.put(1, 0, 0.0);
            newCamMat.put(1, 1, 1417.5028076171875);
            newCamMat.put(1, 2, 524.6199316240309);

            newCamMat.put(2, 0, 0.0);
            newCamMat.put(2, 1, 0.0);
            newCamMat.put(2, 2, 1.0);
        }
        else if( this.camID == 1 ) {
            newCamMat.put(0, 0, 1417.0025634765625);
            newCamMat.put(0, 1, 0.0);
            newCamMat.put(0, 2, 974.1928097073833);

            newCamMat.put(1, 0, 0.0);
            newCamMat.put(1, 1, 1417.5028076171875);
            newCamMat.put(1, 2, 524.6199316240309);

            newCamMat.put(2, 0, 0.0);
            newCamMat.put(2, 1, 0.0);
            newCamMat.put(2, 2, 1.0);
        }
        // ^ getting these values is way easier in python
        return newCamMat;
    }

    // old[[-0.16241381 0.11515442 -0.00715798 0.00620839 -0.0915288 ]]
    private Mat loadDists() {
        Mat dists = new Mat(1, 5, CvType.CV_64F, new Scalar(0));
        if ( this.camID == 0 ) {
            dists.put(0, 0, 0.05140565587875675);
            dists.put(0, 1, -0.22393758599529429);
            dists.put(0, 2, 0.0011717733816520855);
            dists.put(0, 3, -0.0006126942692220965);
            dists.put(0, 4, 0.20549267503932084);
        }
        else if( this.camID == 1 ) {
            dists.put(0, 0, 0.05140565587875675);
            dists.put(0, 1, -0.22393758599529429);
            dists.put(0, 2, 0.0011717733816520855);
            dists.put(0, 3, -0.0006126942692220965);
            dists.put(0, 4, 0.20549267503932084);
        }
        // ^ getting these values is way easier in python
        return dists;
    }



}
