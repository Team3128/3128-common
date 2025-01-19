package common.core.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;
import common.core.controllers.ControllerBase;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.sysid.CmdSysId;
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

    /**
     * Creates an PositionSubsystemBase object.
     * 
     * @param controller Controller for motion control.
     * @param motors The motors of the subsystem.
     */
    public PositionSubsystemBase(ControllerBase controller, NAR_Motor... motors) {
        super(controller, motors);
        
        requireNonNullParam(controller, "controller", "PositionSubsystemBase");
        requireNonNullParam(motors, "leader", "PositionSubsystemBase");
        
        this.motors = Arrays.asList(motors);

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

    /**
     * Sets power to motor.
     * 
     * @param power Setpoint the pivot goes to.
     * @return Command setting pivot setpoint.
     */
    public Command run(double power) {
        return runOnce(()-> motors.forEach(motor -> motor.set(power))).beforeStarting(()-> disable());
    }


    public Command runVolts(double volts) {
        return runOnce(()-> motors.forEach(motor -> motor.setVolts(volts))).beforeStarting(()-> disable());
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
     * Reset measurement position.
     * 
     * @param position Position to reset to.
     * @return Command that resets the pivot position.
     */
    public Command reset(double position) {
        return runOnce(()-> motors.forEach(motor -> motor.resetPosition(position))).beforeStarting(()-> disable());
    }

    public Command reset() {
        return reset(controller.getInputRange()[0]);
    }

    /**
     * Returns current of the first motor.
     */
    public double getCurrent(){
        return motors.get(0).getStallCurrent();
    }

    public void setNeutralMode(Neutral mode) {
        motors.forEach(motor -> motor.setNeutralMode(mode));
    }

    public double getPosition() {
        return motors.get(0).getPosition();
    }

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
            reset(controller.getInputRange()[0]),
            stop()
        ).beforeStarting(()-> disable());
    }

    public Command characterization(double startDelaySecs, double rampRateVoltsPerSec) {
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
        ).beforeStarting(reset(controller.getInputRange()[0]));
    }


}