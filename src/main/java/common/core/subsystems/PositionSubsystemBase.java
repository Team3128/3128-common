package common.core.subsystems;

import java.util.Arrays;
import java.util.function.DoubleSupplier;
import common.core.controllers.ControllerBase;
import common.hardware.motorcontroller.NAR_Motor;
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
public abstract class PositionSubsystemBase extends NAR_PIDSubsystem{

    protected final NAR_Motor leader;

    /**
     * Creates an PositionSubsystemBase object.
     * 
     * @param controller Controller for motion control.
     * @param leader The leader motor which subsequent motors should follow.
     * @param followers The follower motors that follow the leader motor.
     */
    public PositionSubsystemBase(ControllerBase controller, NAR_Motor leader, NAR_Motor... followers) {
        super(controller, leader);
        
        requireNonNullParam(controller, "controller", "PositionSubsystemBase");
        requireNonNullParam(leader, "leader", "PositionSubsystemBase");
        
        this.leader = leader;
        Arrays.stream(followers).forEach((follower)-> follower.follow(this.leader));
        
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
        return runOnce(()-> leader.set(power)).beforeStarting(()-> disable());
    }


    public Command runVolts(double volts) {
        return runOnce(()-> leader.setVolts(volts)).beforeStarting(()-> disable());
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
        return runOnce(()-> leader.resetPosition(position)).beforeStarting(()-> disable());
    }

    public Command reset() {
        return reset(controller.getInputRange()[0]);
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
            waitUntil(()-> (leader.getStallCurrent() > currentLimit)),
            reset(controller.getInputRange()[0]),
            stop()
        ).beforeStarting(()-> disable());
    }

    public Command characterization(double startDelaySecs, double rampRateVoltsPerSec) {
        return new CmdSysId(
            getName(), 
            leader::setVolts, 
            leader::getVelocity, 
            leader::getPosition, 
            startDelaySecs,
            rampRateVoltsPerSec,
            controller.getInputRange()[1], 
            true, 
            this
        ).beforeStarting(reset(controller.getInputRange()[0]));
    }


}