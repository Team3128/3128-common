package common.core.controllers;

import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

public class Controller implements Sendable {

    protected final List<NAR_Motor> motors = new ArrayList<NAR_Motor>();
    protected final PIDFFConfig config;
    protected double tolerance;

    public double setpoint;
    public boolean enabled = false;
    public Type type;
    public double[] inputRange;

    protected DoubleSupplier measurement;
    protected PIDController controller;

    public enum Type{
        POSITION,
        VELOCITY
    }

    public Controller(PIDFFConfig config, Type type) {
        this(config, 0.05, type);
    }

    public Controller(PIDFFConfig config, double tolerance, Type type) {
        this(config, tolerance, type, new double[]{Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY});
    }
        
    public Controller(PIDFFConfig config, Type type, double[] inputRange) {
        this(config, 0.05, type, inputRange);
    }

    public Controller(PIDFFConfig config, double tolerance, Type type, double[] inputRange) {
        this.config = config;
        this.tolerance = tolerance;
        this.type = type;
        this.inputRange = inputRange;

        controller = new PIDController(config.getkP(), config.getkI(), config.getkD());        
        requireNonNullParam(config, "config", "Controller");
    }

    public void addMotor(NAR_Motor motor) {
        motors.add(motor);
    }

    public void setMeasurementSource(NAR_Motor motor){
        if(type == Type.POSITION) measurement = () -> motor.getPosition();
        else measurement = () -> motor.getVelocity();
    }

    public void setSetpoint(double setpoint){
        if(type == Type.POSITION){
            controller.setSetpoint(Math.max(inputRange[0], Math.min(setpoint, inputRange[1])));
        }
        else controller.setSetpoint(setpoint);
    }

    public void setTolerance(double tolerance) {
        controller.setTolerance(tolerance);
    }

    public void useOutput() {
        if (atSetpoint()) {
            disable();
        }
        else if (isEnabled()) {
            final double output = calculatePID(getMeasurement()) + calculateFF(calculatePID(getMeasurement()));
            for (NAR_Motor motor : motors) {
                motor.set(output);
            }
        }
    }

    protected double calculatePID(double measurement){
        return controller.calculate(measurement);
    }

    protected double calculateFF(double pidOutput){
        final double staticGain = !atSetpoint() ? Math.copySign(config.getkS(), pidOutput) : 0;
        final double velocityGain = (type == Type.VELOCITY) ? config.getkV() * getSetpoint() : 0;
        final double gravityGain = config.getkG() * config.getkG_Function().getAsDouble();
        return staticGain + velocityGain + gravityGain;
    }

    public double getMeasurement() {
        return measurement.getAsDouble();
    }
    
    public double getSetpoint() {
        return controller.getSetpoint();
    }

    public boolean atSetpoint() {
        return controller.atSetpoint();
    }
    
    public void reset() {
        controller.reset();
    }

    public void enable() {
        enabled = true;
        controller.reset();
    }

    public void disable() {
        enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }
    
    public void close() {
        controller.close();
    }

    public PIDFFConfig getConfig() {
        return this.config;
    }

    public void enableContinuousInput() {
        controller.enableContinuousInput(inputRange[0], inputRange[1]);
    }

    public double[] getInputRange() {
        return inputRange;
    }

    public Type getType(){
        return type;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("p", config::getkP, config::setkP);
        builder.addDoubleProperty("i", config::getkI, config::setkI);
        builder.addDoubleProperty("d", config::getkD, config::setkD);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }
}
