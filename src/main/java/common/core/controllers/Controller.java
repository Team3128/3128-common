package common.core.controllers;

import java.util.LinkedList;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import common.core.subsystems.NAR_PIDSubsystem;
import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.math.controller.PIDController;

/**
 * Team 3128's streamlined {@link PIDController} class.
 * 
 * @since 2023 CHARGED UP
 * @author Mason Lam, William Yuan
 */
public class Controller extends PIDController {

    private final LinkedList<DoubleConsumer> consumers;
    private DoubleSupplier measurement;

    protected DoubleSupplier kS, kV, kG;
    protected DoubleSupplier kG_Function;
    /**
     * Create a new object to control PID logic for a subsystem.
     * <p>Sets kP, kI, kD, period values.
     * <p>Defaults kS, kV, kG to 0.
     * <p>Defaults kG_Function to 1.
     * <p>Initiliazes consumers.
     * @see LinkedList consumers: includes void functions that accept double values.
     * 
     * @param kP The proportional coefficient of the on board PIDController.
     * @param kI The integral coefficient of the on board PIDController.
     * @param kD The derivative coefficient of the on board PIDController.
     * @param period The PID periodic update rate in seconds.
     */
    public Controller(double kP, double kI, double kD, double period) {
        super(kP, kI, kD, period);
        kS = ()-> 0;
        kV = ()-> 0;
        kG = ()-> 0;
        kG_Function = () -> 1;
        consumers = new LinkedList<DoubleConsumer>();
    }

    /**
     * Create a new object to control PID logic for a subsystem.
     * <p>Sets kP, kI, kD values.
     * <p>Defaults period to 0.02.
     * <p>Defaults kS, kV, kG to 0.
     * <p>Defaults kG_Function to 1.
     * <p>Initiliazes consumers.
     * @see LinkedList consumers: includes void functions that accept double values.
     * 
     * @param kP The proportional coefficient of the on board {@link PIDController}.
     * @param kI The integral coefficient of the on board {@link PIDController}.
     * @param kD The derivative coefficient of the on board {@link PIDController}.
     */
    public Controller(double kP, double kI, double kD) {
        this(kP, kI, kD, 0.02);
    }

    /**
     * Sets the measurement source to be later used during PID logic.
     * @param measurement {@code getMeasurement()} from {@link NAR_PIDSubsystem} as a DoubleSupplier
     */
    public void setMeasurementSource(DoubleSupplier measurement) {
        this.measurement = measurement;
    }

    /**
     * Adds a motor set method to consumers to be later used during PID logic.
     * @see LinkedList consumers: includes void functions that accept double values.
     * @param motor Created {@link NAR_Motor} object.
     */
    public void addMotor(NAR_Motor motor) {
        consumers.add(motor::set);
    }

    /**
     * Adds a useOutput method to consumers to be later used during PID logic.
     * @see LinkedList consumers: includes void functions that accept double values.
     * @param output {@code useOutput()} from {@link NAR_PIDSubsystem} as a DoubleSupplier
     */
    public void addOutput(DoubleConsumer output) {
        consumers.add(output);
    }

    /**
     * Calculates the output based on calculations from the measurement, then gives each DoubleConsumer in consumers the output as an argument.
     * @return calculated output based on calculations from the measurement
     * @see LinkedList consumers: includes void functions that accept double values.
     */
    public double useOutput() {
        if (measurement == null) return 0;
        final double output = calculate(measurement.getAsDouble());
        for (final DoubleConsumer consumer : consumers) {
            consumer.accept(output);
        }
        return output;
    }

    /**
     * Sets the kG_Function to a new DoubleSupplier.
     * @param kG_Function DoubleSupplier with specified logic.
     */
    public void setkG_Function(DoubleSupplier kG_Function) {
        this.kG_Function = kG_Function;
    }

    /**
     * Sets the kS value.
     * @param kS The constant power required to overcome static friction as a double.
     */
    public void setkS(double kS) {
        setkS(()-> kS);
    }

    /**
     * Sets the kS value.
     * @param kS The constant power required to overcome static friction as a DoubleSupplier.
     */
    public void setkS(DoubleSupplier kS) {
        this.kS = kS;
    }

    /**
     * Sets the kV value.
     * @param kV The constant power required to maintain a set velocity as a double.
     */
    public void setkV(double kV) {
        setkV(()-> kV);
    }

    /**
     * Sets the kV value.
     * @param kV The constant power required to maintain a set velocity as a DoubleSupplier.
     */
    public void setkV(DoubleSupplier kV) {
        this.kV = kV;
    }

    /**
     * Sets the kG value.
     * @param kG The constant power required to overcome gravity as a double.
     */
    public void setkG(double kG) {
        this.kG = ()-> kG;
    }

    /**
     * Sets the kG value.
     * @param kG The constant power required to overcome gravity as a DoubleSupplier.
     */
    public void setkG(DoubleSupplier kG) {
        this.kG = kG;
    }

    /**
     * Returns kS.
     * @return returns kS as a double.
     */
    public double getkS() {
        return kS.getAsDouble();
    }

    /**
     * Returns kV.
     * @return returns kV as a double.
     */
    public double getkV() {
        return kV.getAsDouble();
    }
    
    /**
     * Returns kG.
     * @return returns kG as a double.
     */
    public double getkG() {
        return kG.getAsDouble();
    }
}
