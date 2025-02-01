package common.core.subsystems;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import common.core.controllers.ControllerBase;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.shuffleboard.NAR_Shuffleboard;
import common.utility.sysid.CmdSysId;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
    protected DoubleSupplier debugVoltage;

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
     * @param power The power to set the motors to.
     */
    public void run(double power) {
        apply(motor -> motor.set(power));
    }

    /**
     * Command setting power to motors.
     * 
     * @param power The power to set the motors to.
     * @return Command setting the power to the motors.
     */
    public Command runCommand(double power) {
        return applyCommand(motor -> motor.set(power));
    }

    /**
     * Sets voltage to motors.
     * 
     * @param volts The voltage to set the motors to.
     */
    public void runVolts(double volts) {
        System.out.println("Running Volts at " + volts);
        apply(motor -> motor.setVolts(volts));
    }

    /**
     * Command setting voltage to motors.
     * 
     * @param volts The voltage to set the motors to.
     * @return Command setting the voltage to the motors.
     */
    public Command runVoltsCommand(double volts) {
        return applyCommand(motor -> motor.setVolts(volts));
    }

    public double getVolts() {
        return motors.get(0).getAppliedOutput() * 12;
    }

    /**
     * Stops all motors.
     */
    public void stop() {
        run(0);
    }

    /**
     * Command stopping all motors.
     * 
     * @return Command stopping all the motors.
     */
    public Command stopCommand(){
        return runCommand(0);
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
     * Resets measurement position to controller position minimum.
     */
    public void reset() {
        reset(controller.getInputRange()[0]);
    }
    
    /**
     * Command resetting position to controller position minimum.
     * 
     * @return Command resetting the position to the controller position minimum.
     */
    public Command resetCommand() {
        return resetCommand(controller.getInputRange()[0]);
    }

    /**
     * Reset measurement position.
     * 
     * @param position Position to reset to.
     */
    public void reset(double position) {
        apply(motor -> motor.resetPosition(position));
    }

    /**
     * Command resetting position.
     * @param position Position to reset to.
     * @return Command resetting the position.
     */
    public Command resetCommand(double position) {
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
            runCommand(power),
            waitSeconds(delay),
            waitUntil(()-> (getCurrent() > currentLimit)),
            resetCommand(),
            stopCommand()
        ).beforeStarting(()-> disable());
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
            endPosition, 
            true, 
            this
        ).beforeStarting(resetCommand(startPosition));
    }

    @Override
    public void initShuffleboard() {
        super.initShuffleboard();
        NAR_Shuffleboard.addData(getName(), "Current", ()-> motors.get(0).getStallCurrent(), 4, 1);
        NAR_Shuffleboard.addData(getName(), "Voltage", ()-> getVolts(), 4, 2);
        //NAR_Shuffleboard.addData(getName(), "Voltage", ()-> 5, 4, 0);
        NAR_Shuffleboard.addCommand(getName(), "Enable", either(startEnd(()-> startPID(setpoint.getAsDouble()), ()-> disable()), print("DEBUG NOT ON"), debug), 4, 0);
        FFWidgets(controller, 0, 2);
        resetWidget (debug, 5, 0 );
        runVoltsWidgets("Running" ,debug, 4, 3);
    }

    public void FFWidgets(ControllerBase controller, int x, int y) {
        controller.getConfig().setkS(NAR_Shuffleboard.debug(getName(), "kS", controller.getConfig().getkS(), x, y));
        controller.getConfig().setkV(NAR_Shuffleboard.debug(getName(), "kV", controller.getConfig().getkV(), x + 1, y));
        controller.getConfig().setkA(NAR_Shuffleboard.debug(getName(), "kA", controller.getConfig().getkA(), x + 1, y + 1));
        controller.getConfig().setkG(NAR_Shuffleboard.debug(getName(), "kG", controller.getConfig().getkG(), x, y + 1));
        NAR_Shuffleboard.addCommand(getName(), "Characterize", this.characterization(1, 0.5), x+6, y - 2).withSize(1, 1);
    }
}