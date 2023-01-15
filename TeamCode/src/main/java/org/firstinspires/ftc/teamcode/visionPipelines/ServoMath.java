package org.firstinspires.ftc.teamcode.visionPipelines;

public class ServoMath { // the back of the robot is 0 deg
    private static final double SERVO_ZERO = Math.toRadians(30);
    private static final double SERVO_ONE = Math.toRadians(-30);
    private static final double TAU = 2*Math.PI;
    public static double angleWrap(double angle, boolean allowNegative) { //-180 to 180
        angle%=TAU;
        if (angle > Math.PI) angle-=TAU;
        else if (angle < - Math.PI) angle+=TAU;
        if (!allowNegative && angle < 0) angle += TAU;
        return angle;
    }
    public static double servoToRadians(double servo) {
        return Math.toRadians(300) * servo + SERVO_ZERO;
    }
    public static double radiansToServo(double radians) {
        return (angleWrap(radians - SERVO_ZERO, false)) / Math.toRadians(300) ;
    }
}
