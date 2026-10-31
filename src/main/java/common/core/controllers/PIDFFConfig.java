package common.core.controllers;

import java.util.function.DoubleSupplier;

/**
 * The PID and feedforward gains for a mechanism's control loop. Each gain produces volts, in the
 * mechanism's units.
 *
 * <p>Feedforward gains are stored as {@code DoubleSupplier}s so they can be read from a
 * Shuffleboard box while tuning. See {@code README.md} in this package for what each gain does.
 */
public class PIDFFConfig {

    public double kP;
    public double kI;
    public double kD;
    public DoubleSupplier kS;
    public DoubleSupplier kV;
    public DoubleSupplier kA;
    public DoubleSupplier kG;

    /** Multiplier applied to kG in {@link #positionFF} and {@link #velocityFF}, for an angle-dependent load. Defaults to 1. */
    public DoubleSupplier kG_Function = ()-> 1;

    /**
     * Creates a config whose feedforward gains are read from suppliers, such as Shuffleboard boxes.
     */
    public PIDFFConfig(double kP, double kI, double kD, DoubleSupplier kS, DoubleSupplier kV, DoubleSupplier kA, DoubleSupplier kG) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
    }

    /**
     * Creates a config with every gain.
     * @param kP Volts per unit of error.
     * @param kI Volts per unit of error, per second.
     * @param kD Volts per unit/second of change in error.
     * @param kS Volts required to overcome friction.
     * @param kV Volts per unit/second of speed.
     * @param kA Volts per unit/second^2 of acceleration.
     * @param kG Volts required to hold the mechanism against gravity.
     */
    public PIDFFConfig(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        this(kP, kI, kD, ()->kS, ()->kV, ()->kA, ()->kG);
    }

    /**
     * Creates a config with every gain except kA, which the motor controllers do not use.
     * @param kP Volts per unit of error.
     * @param kI Volts per unit of error, per second.
     * @param kD Volts per unit/second of change in error.
     * @param kS Volts to overcome friction.
     * @param kS Volts required to overcome friction.
     * @param kG Volts required to hold the mechanism against gravity.
     */
    public PIDFFConfig(double kP, double kI, double kD, double kS, double kV, double kG) {
        this(kP, kI, kD, kS, kV, 0, kG);
    }

    /**
     * Creates a config with only PID gains; feedforward is 0.
     * @param kP Volts per unit of error.
     * @param kI Volts per unit of error, per second.
     * @param kD Volts per unit/second of change in error.
     */
    public PIDFFConfig(double kP, double kI, double kD) {
        this(kP, kI, kD, 0, 0, 0, 0);
    }

    /**
     * Creates a config with only kP; all other gains are 0.
     * @param kP Volts per unit of error.
     */
    public PIDFFConfig(double kP) {
        this(kP, 0, 0, 0, 0, 0, 0);
    }

    /**
     * Creates a config with every gain set to 0, for a mechanism that runs only open loop.
     */
    public PIDFFConfig() {
        this(0, 0, 0);
    }


    /**
     * Copies every gain, and the gravity function, from another config.
     * @param config The config to copy.
     */
    public void change(PIDFFConfig config){
        this.kP = config.kP;
        this.kI = config.kI;
        this.kD = config.kD;
        this.kS = config.kS;
        this.kG = config.kG;
        this.kA = config.kA;
        this.kV = config.kV;
        this.kG_Function = config.kG_Function;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public void setkI(double kI) {
        this.kI= kI;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    /** Sets kS, the volts required to overcome friction. */
    public void setkS(double kS) {
        setkS(()-> kS);
    }

    /**
     * Sets kS, the volts required to overcome friction, from a supplier read at each use.
     */
    public void setkS(DoubleSupplier kS) {
        this.kS = kS;
    }

    /**
     * Sets kV, the volts per unit/second of speed.
     */
    public void setkV(double kV) {
        setkV(()-> kV);
    }

    /**
     * Sets kV, the volts per unit/second of speed, from a supplier read at each use.
     */
    public void setkV(DoubleSupplier kV) {
        this.kV = kV;
    }

    /**
     * Sets kA, the volts per unit/second^2 of acceleration, from a supplier read at each use.
     */
    public void setkA(DoubleSupplier kA) {
        this.kA = kA;
    }

    /**
     * Sets kG, the volts required to hold the mechanism against gravity.
     */
    public void setkG(double kG) {
        this.kG = ()-> kG;
    }

    /**
     * Sets kG, the volts required to hold the mechanism against gravity, from a supplier read at each use.
     */
    public void setkG(DoubleSupplier kG) {
        this.kG = kG;
    }

    /**
     * Sets the multiplier applied to kG in {@link #positionFF} and {@link #velocityFF}, such as the
     * cosine of an arm's angle when the roboRIO runs the loop. Mechanisms whose loop runs on the
     * motor controller handle arm gravity there and do not need this.
     */
    public void setkG_Function(DoubleSupplier kG_Function) {
        this.kG_Function = kG_Function;
    }

    public double getkP() {
        return kP;
    }

    public double getkI() {
        return kI;
    }

    public double getkD() {
        return kD;
    }

    /**
     * @return The current kS, the volts required to overcome friction.
     */
    public double getkS() {
        return kS.getAsDouble();
    }

    /**
     * @return The current kV, the volts per unit/second of speed.
     */
    public double getkV() {
        return kV.getAsDouble();
    }

    /**
     * @return The current kA, the volts per unit/second^2 of acceleration.
     */
    public double getkA() {
        return kA.getAsDouble();
    }

    /**
     * @return The current kG, the volts required to hold the mechanism against gravity.
     */
    public double getkG() {
        return kG.getAsDouble();
    }

    public DoubleSupplier getkG_Function() {
        return kG_Function;
    }

    /**
     * Calculates the feedforward volts for a mechanism moving to a position when the roboRIO runs its
     * loop: kS in the direction of the applied output, plus kG. kV is omitted because the setpoint is a
     * position, not a speed.
     *
     * @param pidOutput The PID output for this loop, used to determine the sign of kS.
     * @param atSetpoint Whether the mechanism is at the setpoint; kS is then omitted to prevent jitter.
     */
    public double positionFF(double pidOutput, boolean atSetpoint) {
        final double staticGain = atSetpoint ? 0 : Math.copySign(getkS(), pidOutput);
        final double gravityGain = getkG() * getkG_Function().getAsDouble();
        return staticGain + gravityGain;
    }

    /**
     * Calculates the feedforward volts for a mechanism spinning at a speed when the roboRIO runs its
     * loop: kS in the direction of the applied output, kV multiplied by the speed, plus kG.
     *
     * @param setpoint The target speed.
     * @param pidOutput The PID output for this loop, used to determine the sign of kS.
     * @param atSetpoint Whether the mechanism is at speed; kS is then omitted to prevent jitter.
     */
    public double velocityFF(double setpoint, double pidOutput, boolean atSetpoint) {
        final double staticGain = atSetpoint ? 0 : Math.copySign(getkS(), pidOutput);
        final double velocityGain = getkV() * setpoint;
        final double gravityGain = getkG() * getkG_Function().getAsDouble();
        return staticGain + velocityGain + gravityGain;
    }
}
