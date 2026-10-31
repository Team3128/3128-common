package common.hardware.motorcontroller;

/**
 * Values shared by the motor classes: encoder resolutions, CAN report rates and default current
 * limits. These should not be changed.
 * @since 2022 Rapid React
 * @author Mason Lam
 */
public class MotorControllerConstants {

    /** Falcon encoder counts per rotation. */
    public static final double FALCON_ENCODER_RESOLUTION = 2048;
    /** Converts RPM to Phoenix 5 native units per 100 ms on a Falcon. */
    public static final double RPM_TO_FALCON = FALCON_ENCODER_RESOLUTION / 600;
    /** CTRE Mag Encoder counts per rotation. */
    public static final double MAG_ENCODER_RESOLUTION = 4096;
    /** Talon SRX encoder counts per rotation. */
    public static final double TALONSRX_ENCODER_RESOLUTION = 4096;
    /** Converts RPM to Phoenix 5 native units per 100 ms on a Talon SRX. */
    public static final double RPM_TO_TALONSRX = TALONSRX_ENCODER_RESOLUTION / 600;

    /** NEO built-in encoder counts per rotation. */
    public static final double SPARKMAX_ENCODER_RESOLUTION = 42;
    /** Converts RPM to encoder counts per second on a NEO. */
    public static final double SPARKMAX_RPM_TO_NUpS = SPARKMAX_ENCODER_RESOLUTION / 60;

    /** Milliseconds between CAN reports: about every 30 seconds, the closest a Spark allows to off. */
    public static final int NO_PRIORITY = 32000;
    /** Milliseconds between CAN reports. */
    public static final int LOW_PRIORITY = 250;
    /** Milliseconds between CAN reports. */
    public static final int MEDIUM_PRIORITY = 100;
    /** Milliseconds between CAN reports. */
    public static final int HIGH_PRIORITY = 20;
    /** Milliseconds between CAN reports. */
    public static final int MAX_PRIORITY = 10;

    /** {@link #MAX_PRIORITY} in hertz, for Phoenix 6. */
    public static final double MAX_PRIORITY_FREQ = 1.0 / (MAX_PRIORITY / 1000.0);
    /** {@link #HIGH_PRIORITY} in hertz, for Phoenix 6. */
    public static final double HIGH_PRIORITY_FREQ = 1.0 / (HIGH_PRIORITY / 1000.0);
    /** {@link #MEDIUM_PRIORITY} in hertz, for Phoenix 6. */
    public static final double MEDIUM_PRIORITY_FREQ = 1.0 / (MEDIUM_PRIORITY / 1000.0);
    /** {@link #LOW_PRIORITY} in hertz, for Phoenix 6. */
    public static final double LOW_PRIORITY_FREQ = 1.0 / (LOW_PRIORITY / 1000.0);

    /** How long to wait for a Spark to answer a setting, in milliseconds. */
    public static final int canSparkMaxTimeout = 500;

    /** Default stator current limit in amps, until a {@code MotorConfig} sets one. Also used for TalonFXs. */
    public static final int NEO_STATOR_CurrentLimit = 40;
    /** Default stator current limit in amps for a NEO 550. */
    public static final int NEO_STATOR_550CurrentLimit = 20;

    /** Default supply current limit in amps, until a {@code MotorConfig} sets one. Also used for TalonFXs. */
    public static final int NEO_SUPPLY_CurrentLimit = 120;
    /** Default supply current limit in amps for a NEO 550. */
    public static final int NEO_SUPPLY_550CurrentLimit = 40;
}
