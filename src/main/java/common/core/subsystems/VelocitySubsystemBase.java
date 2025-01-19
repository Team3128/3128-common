package common.core.subsystems;

import java.util.Arrays;
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

    protected final NAR_Motor leader;

    /**
     * Creates an VelocitySubsystemBase object.
     * 
     * @param controller Controller for motion control.
     * @param leader The leader motor which subsequent motors should follow.
     * @param followers The follower motors that follow the leader motor.
     */
    public VelocitySubsystemBase(ControllerBase controller, NAR_Motor leader, NAR_Motor... followers) {
        super(controller, Arrays.asList(followers));
        //TODO: Fix this stuff it is compelte garbage
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

    public Command reset() {
        return runOnce(()-> leader.resetPosition(0));
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
        return pidTo(setpoint.getAsDouble());
    }

    /**
     * Returns current of the first motor.
     */
    public double getCurrent(){
        return leader.getStallCurrent();
    }

    public void setNeutralMode(Neutral mode) {
        leader.setNeutralMode(mode);
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
        ).beforeStarting(reset());
    }
}