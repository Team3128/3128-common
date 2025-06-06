package common.hardware.motorcontroller;

/**
 * Constants for motor control / conversion. Should not be changed.
 * @since 2022 Rapid React
 * @author Mason Lam (Blame this guy for any wrong numbers)
 */
public class MotorControllerConstants {

    public static final double FALCON_ENCODER_RESOLUTION = 2048; // CPR
    public static final double RPM_TO_FALCON = FALCON_ENCODER_RESOLUTION / 600; //Nu/100ms
    public static final double MAG_ENCODER_RESOLUTION = 4096; // CPR
    public static final double TALONSRX_ENCODER_RESOLUTION = 4096;
    public static final double RPM_TO_TALONSRX = TALONSRX_ENCODER_RESOLUTION / 600;

    public static final double SPARKMAX_ENCODER_RESOLUTION = 42; // CPR
    public static final double SPARKMAX_RPM_TO_NUpS = SPARKMAX_ENCODER_RESOLUTION / 60; // rotations/min -> counts/sec

    public static final int NO_PRIORITY = 32000;
    public static final int LOW_PRIORITY = 250;
    public static final int MEDIUM_PRIORITY = 100;
    public static final int HIGH_PRIORITY = 20;
    public static final int MAX_PRIORITY = 10;

    public static final double MAX_PRIORITY_FREQ = 1.0 / (MAX_PRIORITY / 1000.0);
    public static final double HIGH_PRIORITY_FREQ = 1.0 / (HIGH_PRIORITY / 1000.0);
    public static final double MEDIUM_PRIORITY_FREQ = 1.0 / (MEDIUM_PRIORITY / 1000.0);
    public static final double LOW_PRIORITY_FREQ = 1.0 / (LOW_PRIORITY / 1000.0);

    public static final int canSparkMaxTimeout = 500; //ms

    public static final int NEO_STATOR_CurrentLimit = 40; //amps
    public static final int NEO_STATOR_550CurrentLimit = 20; //amps
    
    public static final int NEO_SUPPLY_CurrentLimit = 120; //amps
    public static final int NEO_SUPPLY_550CurrentLimit = 40; //amps
}
