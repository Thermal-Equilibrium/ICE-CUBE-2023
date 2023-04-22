package org.firstinspires.ftc.teamcode.Utils;


//detected Pole: Pole{xPixel=352.0, width=109.0, isValidPole=true}0,0

import static org.firstinspires.ftc.teamcode.Math.Controllers.CriticallyDampedPDControl.solveKD;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

import org.firstinspires.ftc.teamcode.RR_quickstart.drive.DriveConstants;

// detected Pole: Pole{xPixel=92.0, width=117.0, isValidPole=true} -0.1,2.5
// detected Pole: Pole{xPixel=562.0, width=113.0, isValidPole=true} -0.4,-1.8
// detected Pole: Pole{xPixel=555.0, width=129.0, isValidPole=true} 1.0793615056637846, -1.8024541968487422
// detected Pole: Pole{xPixel=434.0, width=73.0, isValidPole=true} -5.500307242023125,-1.0340386610282355
// detected Pole: Pole{xPixel=89.0, width=107.0, isValidPole=true}  -0.1701621247931212,  2.7879328083329105
//
public class TestLinearInterpolator {


    public static void main(String[] args) throws Exception {
        double rotation_Kp = 11;
        PIDCoefficients HEADING_PID = new PIDCoefficients(rotation_Kp, 0, solveKD(rotation_Kp, DriveConstants.kV, DriveConstants.gyrationConstant * DriveConstants.kA));
        System.out.println("Kd = " + HEADING_PID.kD);
    }
}
