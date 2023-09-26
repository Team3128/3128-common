package common.core.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import common.core.controllers.Controller;
import common.core.controllers.PController;
import common.core.controllers.VController;
import common.utility.NAR_Shuffleboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem based off of {@link PIDSubsystem} 
 * @since 2023 CHARGED UP
 * @author Mason Lam
 */
public abstract class NAR_PIDSubsystem extends SubsystemBase {
    protected final Controller m_controller;
    protected boolean m_enabled;
    private BooleanSupplier debug;
    private DoubleSupplier setpoint;
    private double min, max;

    /**
     * Creates a new PIDSubsystem.
     *
     * @param controller the PIDController to use
     */
    public NAR_PIDSubsystem(Controller controller) {
        m_controller = controller;
        controller.setMeasurementSource(()-> getMeasurement());
        controller.addOutput(this::useOutput);
        min = Double.NEGATIVE_INFINITY;
        max = Double.POSITIVE_INFINITY;
    }

    @Override
    public void periodic() {
        if (m_enabled) {
            m_controller.useOutput();
        }
    }

    public void initShuffleboard() {
        NAR_Shuffleboard.addComplex(getName(), "PID_Controller", m_controller, 0, 0);

        NAR_Shuffleboard.addData(getName(), "Enabled", ()-> isEnabled(), 1, 0);
        NAR_Shuffleboard.addData(getName(), "Measurement", ()-> getMeasurement(), 1, 1);
        NAR_Shuffleboard.addData(getName(), "Setpoint", ()-> getSetpoint(), 1, 2);

        var debugEntry = NAR_Shuffleboard.addData(getName(), "TOGGLE", false, 2, 0).withWidget("Toggle Button");
        debug = ()-> debugEntry.getEntry().getBoolean(false);
        NAR_Shuffleboard.addData(getName(), "DEBUG", ()-> debug.getAsBoolean(), 2, 1);
        setpoint = NAR_Shuffleboard.debug(getName(), "Debug_Setpoint", 0, 2,2);

        if (m_controller instanceof VController) {
            m_controller.setkS(NAR_Shuffleboard.debug(getName(), "kS", m_controller.getkS(), 3, 0));
            m_controller.setkV(NAR_Shuffleboard.debug(getName(), "kV", m_controller.getkV(), 3, 1));
        }
        if (m_controller instanceof PController) {
            m_controller.setkS(NAR_Shuffleboard.debug(getName(), "kS", m_controller.getkS(), 3, 0));
            m_controller.setkG(NAR_Shuffleboard.debug(getName(), "kG", m_controller.getkG(), 3, 1));
        }
    }

    /**
     * Returns the PIDController object controlling the subsystem
     *
     * @return The PIDController
     */
    public Controller getController() {
        return m_controller;
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     */
    public void setTolerance(double positionTolerance) {
        m_controller.setTolerance(positionTolerance);
    }

    /**
     * Sets constraints for the setpoint of the PID subsystem.
     * @param min The minimum setpoint for the subsystem
     * @param max The maximum setpoint for the subsystem
     */
    public void setConstraints(double min, double max) {
        this.min = min;
        this.max = max;
    }

    /**
     * Sets the function returning the value multiplied against kG
     * @param kG_Function the function multiplied to kG
     */
    public void setkG_Function(DoubleSupplier kG_Function) {
        m_controller.setkG_Function(kG_Function);
    }

    /**
     * Sets the setpoint for the subsystem.
     *
     * @param setpoint the setpoint for the subsystem
     */
    public void startPID(double setpoint) {
        enable();
        m_controller.setSetpoint(MathUtil.clamp(debug.getAsBoolean() ? this.setpoint.getAsDouble() : setpoint, min, max));
    }

    /**
     * Enables continuous input.
     *
     * <p>Rather then using the max and min input range as constraints, it considers them to be the
     * same point and automatically calculates the shortest route to the setpoint.
     *
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        m_controller.enableContinuousInput(minimumInput, maximumInput);
    }

    /**
     * Returns the current setpoint of the subsystem.
     *
     * @return The current setpoint
     */
    public double getSetpoint() {
        return m_controller.getSetpoint();
    }

    /**
     * Returns true if subsystem is at setpoint, false if not
     *
     * @return If subsystem is at setpoint
     */
    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    /**
     * Uses the output from the PIDController.
     *
     * @param output the output of the PIDController
     */
    protected abstract void useOutput(double output);

    /**
     * Returns the measurement of the process variable used by the PIDController.
     *
     * @return the measurement of the process variable
     */
    protected abstract double getMeasurement();

    /** Enables the PID control. Resets the controller. */
    public void enable() {
        m_enabled = true;
        m_controller.reset();
    }

    /** Disables the PID control. Sets output to zero. */
    public void disable() {
        m_enabled = false;
        useOutput(0);
    }

    /**
     * Returns whether the controller is enabled.
     *
     * @return Whether the controller is enabled.
     */
    public boolean isEnabled() {
        return m_enabled;
    }
}
