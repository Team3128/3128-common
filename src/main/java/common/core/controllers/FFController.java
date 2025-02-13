package common.core.controllers;

import common.hardware.motorcontroller.NAR_Motor;

public class FFController extends ControllerBase {

    /**
     * Setpoint types
     */
    public enum Type {
        VELOCITY,
        POSITION
    }

    private final Type type;

    /**
     * Create a new object to control PID + FF logic for a subsystem.
     * <p>Sets kP, kI, kD, kS, kV, kG, period values.
     * 
     * @param config PIDFFConfig object containing PID and Feedforward constants.
     * @param type The type of setpoint used by the controller
     * @param period The controller's update rate in seconds. Must be non-zero and positive.
     */
    public FFController(PIDFFConfig config, Type type, double period) {
        super(config, period);
        this.type = type;
    }

    /**
     * Create a new object to control PID + FF logic for a subsystem.
     * <p>Sets kP, kI, kD, kS, kV, kG values.
     * 
     * @param config PIDFFConfig object containing PID and Feedforward constants.
     * @param type The type of setpoint used by the controller
     */
    public FFController(PIDFFConfig config, Type type) {
        this(config, type, 0.02);
    }

    /**
     * Sets a motor's velocity/position as the measurement source
     * @param motor {@link NAR_Motor} motor type.
     */
    public void setMeasurementSource(NAR_Motor motor) {
        if (type == Type.VELOCITY) {
            setMeasurementSource(()-> motor.getVelocity());
            return;
        }
        setMeasurementSource(()-> motor.getPosition());
    }

    public void configureFeedback(NAR_Motor motor) {
        setMeasurementSource(motor);
        addMotor(motor);
    }

    /**
     * Returns the PID output of the controller.
     * @param measurement the current measurement of the process variable.
     * @return The controller output due to PID terms.
     */
    @Override
    public double calculatePID(double measurement) {
        return 0;
    }

    /**
     * Returns the Feed Forward output of the controller.
     * <p>Uses kS, kG, and optionally kV
     * @param pidOutput the current measurement of the process variable.
     * @return The controller output due to Feed Forward terms.
     */
    @Override
    public double calculateFF(double pidOutput) {
        final double staticGain = !atSetpoint() ? Math.copySign(getConfig().getkS(), getError()) : 0;
        final double velocityGain = (type == Type.VELOCITY) ? getConfig().getkV() * getSetpoint() : 0;
        final double gravityGain = getConfig().getkG() * getConfig().getkG_Function().getAsDouble();
        return staticGain + velocityGain + gravityGain;
    }

    /**
     * Returns the type of setpoint used by the controller
     * @return {@link Type}: VELOCITY, POSITION, NONE
    */
    public Type getType() {
        return type;
    }
    
}
