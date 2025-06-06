package common.core.subsystems;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import common.core.controllers.ControllerBase;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.sysid.CmdSysId;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

/**
 * Team 3128's Velocity Subsystem Base.
 * Used for subsystem utilizing velocity measurements for PID control.
 * 
 * @since 2024 Crescendo
 * @author Teja Yaramada
 */
public abstract class VelocitySubsystemBase extends NAR_PIDSubsystem implements NAR_Subsystem {

    protected final List<NAR_Motor> motors;

    /**
     * Creates an VelocitySubsystemBase object.
     * 
     * @param controller Controller for motion control.
     * @param motors The motors of the subsystem.
     */
    public VelocitySubsystemBase(ControllerBase controller, NAR_Motor... motors) {
        super(controller, List.of(motors));
        
        requireNonNullParam(controller, "controller", "VelocitySubsystemBase");
        requireNonNullParam(motors, "motors", "VelocitySubsystemBase");
        
        this.motors = List.of(motors);

        configMotors();
        configController();
    }

    /**
     * Configure motor settings.
     */
    protected abstract void configMotors();

    /**
     * Configure controller settings.
     */
    protected abstract void configController();

    protected void apply(Consumer<NAR_Motor> action) {
        disable();
        motors.forEach(action);
    }

    protected Command applyCommand(Consumer<NAR_Motor> action) {
        return runOnce(()-> apply(action));
    }

    /**
     * Sets power to motor.
     * 
     * @param power Setpoint the pivot goes to.
     */
    public void run(double power) {
        apply(motor -> motor.set(power));
    }
    public Command runCommand(double power) {
        return applyCommand(motor -> motor.set(power));
    }

    public void runVolts(double volts) {
        apply(motor -> motor.setVolts(volts));
    }
    public Command runVoltsCommand(double volts) {
        return applyCommand(motor -> motor.setVolts(volts));
    }

    public double getVolts() {
        return motors.get(0).getAppliedOutput() * 12;
    }

    /**
     * Stops all motors in the subsystem.
     */
    public void stop() {
        disable();
        run(0);
    }
    public Command stopCommand(){
        return runOnce(()-> stop());
    }

    /**
     * Sets controller setpoint and enables controller.
     * 
     * @param setpoint Setpoint the pivot goes to.
     * @return Command setting pivot setpoint.
     */
    public Command pidTo(double setpoint) {
        return runOnce(()-> startPID(setpoint));
    }

    /**
     * Sets controller setpoint and enables controller.
     * 
     * @param setpoint Setpoint the pivot goes to.
     * @return Command setting pivot setpoint.
     */
    public Command pidTo(DoubleSupplier setpoint) {
        return runOnce(()-> startPID(setpoint.getAsDouble()));
    }

    /**
     * Reset measurement position to controller position minimum.
     */
    public void reset() {
        apply(motor -> motor.resetPosition(controller.getInputRange()[0]));
    }
    public Command resetCommand() {
        return applyCommand(motor -> motor.resetPosition(controller.getInputRange()[0]));
    }

    /**
     * Returns current of the first motor.
     */
    public double getCurrent(){
        return motors.get(0).getStallCurrent();
    }

    /**
     * Set the neutral mode for all motors in the mechanism.
     * 
     * @param mode The neutral mode to set to.
     */
    public void setNeutralMode(Neutral mode) {
        apply(motor -> motor.setNeutralMode(mode));
    }

    /**
     * Get the position of the mechanism relative to its reset.
     * 
     * @return The position of the first motor.
     */
    public double getPosition() {
        return motors.get(0).getPosition();
    }

    /**
     * Get the velocity of the mechanism.
     * 
     * @return The velocity of the first motor.
     */
    public double getVelocity() {
        return motors.get(0).getVelocity();
    }

    public Command characterization(double startDelaySecs, double rampRateVoltsPerSec) {
        return characterization(startDelaySecs, rampRateVoltsPerSec, controller.getInputRange()[0], controller.getInputRange()[1]);
    }

    public Command characterization(double startDelaySecs, double rampRateVoltsPerSec, double startPosition, double endPosition) {
        return new CmdSysId(
            getName(), 
            this::runVolts, 
            this::getVelocity, 
            this::getPosition, 
            startDelaySecs,
            rampRateVoltsPerSec,
            controller.getInputRange()[1], 
            true, 
            this
        ).beforeStarting(resetCommand());
    }
}