package common.core.controllers;

/**
 * Stores PID and feedforward constants.
 */
public class PIDFFConfig {

    public double kP;
    public double kI;
    public double kD;
    public double kS;
    public double kV;
    public double kA;
    public double kG;

    /**
     * Creates a config to set PID and feedforward constants.
     * @param kP The proportional coefficient.
     * @param kI The integral coefficient.
     * @param kD The derivative coefficient.
     * @param kS The static gain.
     * @param kV The velocity gain.
     * @param kA The acceleration gain.
     * @param kG The gravity gain.
     */
    public PIDFFConfig(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
    }

    /**
     * Creates a config to set PID and feedforward constants.
     * @param kP The proportional coefficient.
     * @param kI The integral coefficient.
     * @param kD The derivative coefficient.
     * @param kS The static gain.
     * @param kV The velocity gain.
     * @param kG The gravity gain.
     */
    public PIDFFConfig(double kP, double kI, double kD, double kS, double kV, double kG) {
        this(kP, kI, kD, kS, kV, 0, kG);
    }

    /**
     * Creates a config to set PID constants.
     * @param kP The proportional coefficient.
     * @param kI The integral coefficient.
     * @param kD The derivative coefficient.
     */
    public PIDFFConfig(double kP, double kI, double kD) {
        this(kP, kI, kD, 0, 0, 0, 0);
    }

    /**
     * Creates an empty PID config.
     */
    public PIDFFConfig() {
        this(0, 0, 0);
    }
}
