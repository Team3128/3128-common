package common.core.controllers;

import java.util.function.DoubleSupplier;

/**
 * The gains (tuning numbers) for a mechanism's control loop: PID plus feedforward.
 *
 * <h2>What the gains do</h2>
 * Every gain turns something into volts for the motor. Units are whatever the mechanism measures
 * in, e.g. meters for an elevator or degrees for an arm.
 *
 * <p><b>Feedback (PID)</b> pushes based on how far off the mechanism is, the error:
 * <ul>
 * <li><b>kP</b>: volts per unit of error. The main gain: bigger pushes harder, too big
 *     overshoots and wobbles.</li>
 * <li><b>kI</b>: volts per unit of error, per second it's been off. Slowly builds up to fix a
 *     small error that won't go away. Usually 0.</li>
 * <li><b>kD</b>: volts per unit/second the error is changing. Brakes as the mechanism closes in,
 *     to reduce overshoot. Often 0.</li>
 * </ul>
 *
 * <p><b>Feedforward</b> pushes based on what the mechanism needs, before there's any error:
 * <ul>
 * <li><b>kS</b>: volts to overcome friction and get it moving at all.</li>
 * <li><b>kV</b>: volts per unit/second of speed, to keep it moving at that speed. The main gain
 *     for a flywheel.</li>
 * <li><b>kA</b>: volts per unit/second^2 of acceleration. Not used by the motor controllers.</li>
 * <li><b>kG</b>: volts to hold it up against gravity: constant for an elevator, times the cosine
 *     of the angle for an arm, so it's strongest when the arm is level.</li>
 * </ul>
 *
 * <p>Good feedforward does most of the work and leaves PID only small errors to fix, which makes
 * a mechanism fast and smooth. Tune feedforward first, e.g. with SysId, then add kP.
 *
 * <p>Feedforward gains are stored as {@code DoubleSupplier}s rather than numbers so they can
 * come from a Shuffleboard box while tuning: type in a new number, and the next read gets the new
 * value.
 */
public class PIDFFConfig {

    public double kP;
    public double kI;
    public double kD;
    public DoubleSupplier kS;
    public DoubleSupplier kV;
    public DoubleSupplier kA;
    public DoubleSupplier kG;

    /** Multiplies kG in {@link #positionFF} and {@link #velocityFF}, e.g. for an angle-dependent load. Default 1. */
    public DoubleSupplier kG_Function = ()-> 1;

    /**
     * Creates a config whose feedforward gains are read from suppliers, e.g. Shuffleboard boxes.
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
     * @param kS Volts to overcome friction.
     * @param kV Volts per unit/second of speed.
     * @param kA Volts per unit/second^2 of acceleration.
     * @param kG Volts to hold it against gravity.
     */
    public PIDFFConfig(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        this(kP, kI, kD, ()->kS, ()->kV, ()->kA, ()->kG);
    }

    /**
     * Creates a config with every gain but kA, which the motor controllers don't use.
     * @param kP Volts per unit of error.
     * @param kI Volts per unit of error, per second.
     * @param kD Volts per unit/second of change in error.
     * @param kS Volts to overcome friction.
     * @param kV Volts per unit/second of speed.
     * @param kG Volts to hold it against gravity.
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
     * Creates a config with only kP; everything else is 0.
     * @param kP Volts per unit of error.
     */
    public PIDFFConfig(double kP) {
        this(kP, 0, 0, 0, 0, 0, 0);
    }

    /**
     * Creates a config with every gain 0, e.g. for a mechanism you only run open loop.
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

    /** Sets kS, the volts to overcome friction. */
    public void setkS(double kS) {
        setkS(()-> kS);
    }

    /**
     * Sets kS, the volts to overcome friction, read from a supplier each time it's used.
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
     * Sets kV, the volts per unit/second of speed, read from a supplier each time it's used.
     */
    public void setkV(DoubleSupplier kV) {
        this.kV = kV;
    }

    /**
     * Sets kA, the volts per unit/second^2 of acceleration, read from a supplier each time it's used.
     */
    public void setkA(DoubleSupplier kA) {
        this.kA = kA;
    }

    /**
     * Sets kG, the volts to hold it against gravity.
     */
    public void setkG(double kG) {
        this.kG = ()-> kG;
    }

    /**
     * Sets kG, the volts to hold it against gravity, read from a supplier each time it's used.
     */
    public void setkG(DoubleSupplier kG) {
        this.kG = kG;
    }

    /**
     * Sets what kG is multiplied by in {@link #positionFF} and {@link #velocityFF}, e.g. the
     * cosine of an arm's angle when the roboRIO runs the loop. Mechanisms whose loop runs on the
     * motor controller handle arm gravity there instead, so don't need this.
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
     * @return kS, the volts to overcome friction, right now.
     */
    public double getkS() {
        return kS.getAsDouble();
    }

    /**
     * @return kV, the volts per unit/second of speed, right now.
     */
    public double getkV() {
        return kV.getAsDouble();
    }

    /**
     * @return kA, the volts per unit/second^2 of acceleration, right now.
     */
    public double getkA() {
        return kA.getAsDouble();
    }

    /**
     * @return kG, the volts to hold it against gravity, right now.
     */
    public double getkG() {
        return kG.getAsDouble();
    }

    public DoubleSupplier getkG_Function() {
        return kG_Function;
    }

    /**
     * Feedforward volts for a mechanism moving to a position, when the roboRIO runs its loop:
     * kS in the direction it's being pushed, plus kG. There's no kV, since the setpoint is a
     * position, not a speed.
     *
     * @param pidOutput This loop's PID output, used to pick kS's direction.
     * @param atSetpoint Whether it's already there; kS is left out then, so it doesn't jitter.
     */
    public double positionFF(double pidOutput, boolean atSetpoint) {
        final double staticGain = atSetpoint ? 0 : Math.copySign(getkS(), pidOutput);
        final double gravityGain = getkG() * getkG_Function().getAsDouble();
        return staticGain + gravityGain;
    }

    /**
     * Feedforward volts for a mechanism spinning at a speed, when the roboRIO runs its loop:
     * kS in the direction it's being pushed, kV times the speed, plus kG.
     *
     * @param setpoint The target speed.
     * @param pidOutput This loop's PID output, used to pick kS's direction.
     * @param atSetpoint Whether it's already at speed; kS is left out then, so it doesn't jitter.
     */
    public double velocityFF(double setpoint, double pidOutput, boolean atSetpoint) {
        final double staticGain = atSetpoint ? 0 : Math.copySign(getkS(), pidOutput);
        final double velocityGain = getkV() * setpoint;
        final double gravityGain = getkG() * getkG_Function().getAsDouble();
        return staticGain + velocityGain + gravityGain;
    }
}
