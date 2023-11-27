package common.swerveNeo;

import common.swerveNeo.SwerveModule.ModuleType;
import edu.wpi.first.math.util.Units;;

/**
 * Team 3128's defaults
 */
public class SwerveConstants {
    public static ModuleType moduleType = ModuleType.NEO;
    public static double maxSpeed = 4.5;
    public static double maxAcceleration = 2.3;
    public static double wheelDiameter = Units.inchesToMeters(4);
    public static double wheelCircumference = wheelDiameter * Math.PI;

    public static String canBus = "";

    public static double angleKP = 0.15;
    public static double angleKI = 0;
    public static double angleKD = 0;

    public static double angleGearRatio = 150.0 / 7.0;
    public static int angleLimit = 30;
    public static boolean angleMotorInvert = false;

    public static double driveKS = 0.19255;
    public static double driveKV = 2.4366;
    public static double driveKA = 0.34415;

    public static double driveKP = 4e-5;
    public static double driveKI = 0;
    public static double driveKD = 0;

    public static double driveGearRatio = 6.75;
    public static int driveLimit = 40;
    public static boolean driveMotorInvert = false;

    public static boolean canCoderInvert = false;

    //Falcon constants we don't use
    public static int angleContinuousCurrentLimit = 25;
    public static int anglePeakCurrentLimit = 40;
    public static double anglePeakCurrentDuration = 0.1;
    public static boolean angleEnableCurrentLimit = true;

    public static int driveContinuousCurrentLimit = 35;
    public static int drivePeakCurrentLimit = 60;
    public static double drivePeakCurrentDuration = 0.1;
    public static boolean driveEnableCurrentLimit = true;

    public static double driveKF = 0;
    public static double angleKF = 0;

    public static double closedLoopRamp = 0.0;
}
