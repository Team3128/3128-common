package common.core.controllers;

import java.util.function.DoubleSupplier;

/**
 * Stores PID and feedforward constants.
 */
public class PIDFFConfig {

    public double kP;
    public double kI;
    public double kD;
    public DoubleSupplier kS;
    public DoubleSupplier kV;
    public DoubleSupplier kA;
    public DoubleSupplier kG;

    public DoubleSupplier kG_Function = ()-> 1;

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
        this(kP, kI, kD, ()->kS, ()->kV, ()->kA, ()->kG);
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
     * Creates a config to set a P constant.
     * @param kP The proportional coefficient.
     */
    public PIDFFConfig(double kP) {
        this(kP, 0, 0, 0, 0, 0, 0);
    }

    /**
     * Creates an empty PID config.
     */
    public PIDFFConfig() {
        this(0, 0, 0);
    }


    /**
     * Sets the static gain.
     * @param kS The constant power required to overcome static friction as a double.
     */
    public void setkS(double kS) {
        setkS(()-> kS);
    }

    /**
     * Sets the static gain.
     * @param kS Modifies kS based on the function.
     */
    public void setkS(DoubleSupplier kS) {
        this.kS = kS;
    }

    /**
     * Sets the velocity gain.
     * @param kV The constant power required to maintain a set velocity as a double.
     */
    public void setkV(double kV) {
        setkV(()-> kV);
    }

    /**
     * Sets the velocity gain.
     * @param kV Modifies kV based on the function.
     */
    public void setkV(DoubleSupplier kV) {
        this.kV = kV;
    }

    /**
     * Sets the velocity gain.
     * @param kA Modifies kV based on the function.
     */
    public void setkA(DoubleSupplier kA) {
        this.kA = kA;
    }

    /**
     * Sets the gravity gain.
     * @param kG The constant power required to overcome gravity as a double.
     */
    public void setkG(double kG) {
        this.kG = ()-> kG;
    }

    /**
     * Sets the gravity gain.
     * @param kG Modifies kG based on the function.
     */
    public void setkG(DoubleSupplier kG) {
        this.kG = kG;
    }

    /**
     * Sets a function to modify gravity gain based on another factor.
     * <p>Example use would be a Pivot which would have gravity gain dependent on angle.
     * @param kG_Function DoubleSupplier with specified logic.
     */
    public void setkG_Function(DoubleSupplier kG_Function) {
        this.kG_Function = kG_Function;
    }

    /**
     * Returns static gain.
     * @return returns kS as a double.
     */
    public double getkS() {
        return kS.getAsDouble();
    }

    /**
     * Returns velocity gain.
     * @return returns kV as a double.
     */
    public double getkV() {
        return kV.getAsDouble();
    }

    /**
     * Returns acceleration gain.
     * @return returns kA as a double.
     */
    public double getkA() {
        return kA.getAsDouble();
    }
    
    /**
     * Returns gravity gain.
     * @return returns kG as a double.
     */
    public double getkG() {
        return kG.getAsDouble();
    }

    public DoubleSupplier getkG_Function() {
        return kG_Function;
    }
}
