package common.core.subsystems;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import common.core.controllers.ControllerBase;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.sysid.CmdSysId;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;
import static edu.wpi.first.wpilibj2.command.Commands.*;

/**
 * Team 3128's Position Subsystem Base.
 * Used for subsystem utilizing position measurements for PID control.
 * 
 * @since 2024 Crescendo
 * @author Teja Yaramada
 */
public abstract class PositionSubsystemBase extends NAR_PIDSubsystem implements NAR_Subsystem {

    protected final List<NAR_Motor> motors;
    protected final SimpleMotorFeedforward ff;

    /**
     * Creates an PositionSubsystemBase object.
     * 
     * @param controller Controller for motion control.
     * @param motors The motors of the subsystem.
     */
    public PositionSubsystemBase(ControllerBase controller, NAR_Motor... motors) {
        super(controller, List.of(motors));
        ff = new SimpleMotorFeedforward(controller.getConfig().getkS(), controller.getConfig().getkV(), controller.getConfig().getkA());
        
        requireNonNullParam(controller, "controller", "PositionSubsystemBase");
        requireNonNullParam(motors, "motors", "PositionSubsystemBase");
        
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
     * Sets power to motors.
     * 
     * @param power The power to command to the motors.
     * @return Command setting the power to the motors.
     */
    public Command run(double power) {
        return applyCommand(motor -> motor.set(power));
    }

    /**
     * Sets voltage to motors.
     * 
     * @param volts The voltage to command to the motors.
     * @return Command setting the voltage to the motors.
     */
    public Command runVolts(double volts) {
        return applyCommand(motor -> motor.setVolts(volts));
    }

    /**
     * Stops all motors in the subsystem.
     * 
     * @return Command stopping the subsystem motors.
     */
    public Command stop(){
        return run(0);
    }

    /**
     * Sets controller setpoint and enables controller.
     * 
     * @param setpoint Setpoint the pivot goes to.
     * @return Command setting pivot setpoint.
     */
    public Command pidTo(double setpoint) {
        return runOnce(()-> startPID(setpoint))
        ;
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
     * 
     * @return Command resetting the position to the controller input minimum.
     */
    public Command reset() {
        return reset(controller.getInputRange()[0]);
    }

    /**
     * Reset measurement position.
     * 
     * @param position Position to reset to.
     * @return Command resetting the position.
     */
    public Command reset(double position) {
        return applyCommand(motor -> motor.resetPosition(position));
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

    /**
     * Homing command for the subsystem.
     * 
     * @param power Power the motor is run at.
     * @param delay Delay before current check starts.
     * @param currentLimit Current limit for homing.
     * @return Command that homes the subsystem.
     */
    public Command homing(double power, double delay, double currentLimit){
        return sequence(
            run(power),
            waitSeconds(delay),
            waitUntil(()-> (getCurrent() > currentLimit)),
            reset(),
            stop()
        ).beforeStarting(()-> disable());
    }

    public Command charicterization(double startDelaySecs, double rampRateVoltsPerSec) {
        return characterization(startDelaySecs, rampRateVoltsPerSec, controller.getInputRange()[0], controller.getInputRange()[1]);
    }

    public Command characterization(double startDelaySecs, double rampRateVoltsPerSec, double startPosition, double endPosition) {
        return new CmdSysId(
            getName(), 
            volts -> apply(motor -> motor.setVolts(volts)), 
            this::getVelocity, 
            this::getPosition, 
            startDelaySecs,
            rampRateVoltsPerSec,
            endPosition, 
            true, 
            this
        ).beforeStarting(reset(startPosition));
    }
}