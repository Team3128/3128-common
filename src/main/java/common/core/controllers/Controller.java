package common.core.controllers;

import java.util.LinkedList;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Team 3128's streamlined {@link PIDController} class.
 * 
 * @since 2023 CHARGED UP
 * @author Mason Lam, William Yuan
 */
public class Controller extends PIDController {

    /**
     * Setpoint types
     */
    public enum Type {
        VELOCITY,
        POSITION,
        DEFAULT;
    }

    private final LinkedList<DoubleConsumer> consumers;
    private final Type type;
    private DoubleSupplier measurement;

    protected DoubleSupplier kS, kV, kG;
    protected DoubleSupplier kG_Function;

    /**
     * Create a new object to control PID + FF logic for a subsystem.
     * <p>Sets kP, kI, kD, kS, kV, kG, period values.
     * 
     * @param kP The proportional coefficient.
     * @param kI The integral coefficient.
     * @param kD The derivative coefficient.
     * @param kS The static gain.
     * @param kV The velocity gain.
     * @param kG The gravity gain.
     * @param type The type of setpoint used by the controller
     * @param period The controller's update rate in seconds. Must be non-zero and positive.
     */
    public Controller(double kP, double kI, double kD, double kS, double kV, double kG, Type type, double period) {
        super(kP, kI, kD, period);
        consumers = new LinkedList<DoubleConsumer>();
        this.kS = ()-> kS;
        this.kV = ()-> kV;
        this.kG = ()-> kG;
        kG_Function = ()-> 1;
        this.type = type;
    }

    /**
     * Create a new object to control PID logic for a subsystem.
     * <p>Sets kP, kI, kD, period values.
     * 
     * @param kP The proportional coefficient.
     * @param kI The integral coefficient.
     * @param kD The derivative coefficient.
     * @param period The controller's update rate in seconds. Must be non-zero and positive.
     */
    public Controller(double kP, double kI, double kD, double period) {
        this(kP, kI, kD, 0, 0, 0, Type.DEFAULT, period);
    }

    /**
     * Create a new object to control PID logic for a subsystem.
     * <p>Sets kP, kI, kD values.
     * 
     * @param kP The proportional coefficient.
     * @param kI The integral coefficient.
     * @param kD The derivative coefficient.
     */
    public Controller(double kP, double kI, double kD) {
        this(kP, kI, kD, 0.02);
    }

    /**
     * Sets a motor's velocity/position as the measurement source
     * @param motor {@link NAR_Motor} motor type.
     */
    public void setMotorMeasurementSource(NAR_Motor motor) {
        if (type == Type.VELOCITY) {
            setMeasurementSource(()-> motor.getVelocity());
            return;
        }
        setMeasurementSource(()-> motor.getPosition());
    }

    /**
     * Sets the measurement source for PID control.
     * @param measurement Returns the current state of the system.
     */
    public void setMeasurementSource(DoubleSupplier measurement) {
        this.measurement = measurement;
    }

    /**
     * Adds a motor for controller to send output
     * @param motor {@link NAR_Motor} motor type.
     */
    public void addMotor(NAR_Motor motor) {
        addOutput(motor::set);
    }

    /**
     * Adds a motor for controller to send output
     * @param motor {@link MotorController} motor type.
     */
    public void addMotor(MotorController motor) {
        addOutput(motor::set);
    }
    

    /**
     * Adds an output source for controller to send output
     * @param output Method that takes the controller's output
     */
    public void addOutput(DoubleConsumer output) {
        consumers.add(output);
    }

    /**
     * Returns the next output of the controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The next controller output.
     */
    @Override
    public double calculate(double measurement) {
        final double PID_OUTPUT = super.calculate(measurement);
        if (type == Type.DEFAULT) return PID_OUTPUT;
        
        double ff_output = Math.copySign(getkS(), PID_OUTPUT);
        
        if (type == Type.VELOCITY) {
            ff_output += getkV() * getSetpoint();
            return ff_output + PID_OUTPUT;
        }
        ff_output += getkG() * kG_Function.getAsDouble();
        return ff_output + PID_OUTPUT;
    }

    /**
     * Calculates and sends output.
     * @return Calculated output.
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
     * Returns the type of setpoint used by the controller
     * @return {@link Type}: VELOCITY, POSITION, NONE
     */
    public Type getType() {
        return type;
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
     * Returns gravity gain.
     * @return returns kG as a double.
     */
    public double getkG() {
        return kG.getAsDouble();
    }
}
