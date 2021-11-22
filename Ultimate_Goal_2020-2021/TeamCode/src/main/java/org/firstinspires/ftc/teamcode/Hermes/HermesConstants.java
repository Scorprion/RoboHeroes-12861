package org.firstinspires.ftc.teamcode.Hermes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.PIDCoeffs;

@Config
public class HermesConstants {
    // NOTE: All measurements of length are in inches

    public static double ROBOT_CONTROL_SPEED = 0.7;

    public static final double WHEEL_RADIUS = Math.PI * 75 * 5 / 127;  // wheel radius
    public static final double COUNTS_PER_INCH = 560 * (1 / WHEEL_RADIUS); // 560 counts per rev
    public static final double WHEEL_RAD_PER_SECOND = 3. * WHEEL_RADIUS; // * ROBOT_CONTROL_SPEED;   // (300 is theoretical max) assuming 182.5 is the max load rpm of the motors (based on estimate) -> 3.042 rps

    public static final double AVG_RAD = 1 / (4 * COUNTS_PER_INCH);  // Added the division of counts per inch to avoid the extra calculation

    public static double RADIUS_X = 10;
    public static double RADIUS_Y = 10;

    public static double FInit = 32767 / 2600.;  // 2600 max counts rpm (theoretical max: 5 * 560)
    public static double PInit = 0.1 * FInit;
    public static double IInit = 0.1 * PInit;
    public static double DInit = 0;

    public static PIDFCoefficients coeffs = new PIDFCoefficients(PInit, IInit, DInit, FInit, MotorControlAlgorithm.PIDF);

    /*
    public static PIDCoeffs xPID = new PIDCoeffs(PInit, IInit, 0, FInit);
    public static PIDCoeffs yPID = new PIDCoeffs(PInit, IInit, 0, FInit);
    public static PIDCoeffs angPID = new PIDCoeffs(2, 1, 4, FInit);
    */

}
