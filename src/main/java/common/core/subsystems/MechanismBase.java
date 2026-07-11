package common.core.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import common.core.controllers.ControllerBase;
import common.core.controllers.PositionController;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;
import common.utility.sysid.NAR_SysIdCommand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class MechanismBase extends SubsystemBase {

    protected ControllerBase controller;
    protected NAR_Motor[] motors;
    private double safetyThresh;
    private Timer safetyTimer = new Timer();

    protected BooleanSupplier debug;
    protected DoubleSupplier setpoint;

    protected static List<MechanismBase> instances = new ArrayList<>();

    public MechanismBase() {}

    public MechanismBase(ControllerBase controller, MotorConfig m_config, NAR_Motor... motors) {
        this.controller = controller;
        this.motors = motors;
        this.safetyThresh = 5;

        requireNonNullParam(motors, "motors", "MechanismBase");

        if (controller != null) {
            controller.setMeasurementSource(motors[0]);
            for (int i = 0; i < motors.length; i++) {
                controller.addMotor(motors[i]);
                motors[i].configMotor(m_config);
            }
        }
    }

    public MechanismBase(MotorConfig m_config, NAR_Motor... motors) {
        this(null, m_config, motors);
    }

    /**
     * This is an extendable implementation of the singleton pattern<br><br>
     * 
     * When getting a mechanism instance whose class name is [CLASS_NAME], use the following code:<br>
     * <strong>[CLASS_NAME] mechanism = [CLASS_NAME].getInstance([CLASS_NAME].class);</strong><br>
     * 
     * @param type [MECHANISM_CLASS_NAME].class
     * @return a singleton instance of the mechanism
     */
    public static <T extends MechanismBase> T getInstance(Class<T> type) {
        for (MechanismBase instance : instances) {
            if (type.isInstance(instance)) {
                return type.cast(instance);
            }
        }
        MechanismBase instance = type.cast(new MechanismBase());
        instances.add(instance);
        return type.cast(instance);
    }

    @Override
    public void periodic() {
        if (controller != null && controller.isEnabled()) {
            controller.useOutput();
            if (safetyTimer.hasElapsed(safetyThresh)) onSafetyTimeout();
            if (atSetpoint()) safetyTimer.restart();
        }

        NAR_Shuffleboard.addData(getName(), "Velocity", motors[0].getVelocity(), 5, 1);
    }

    /**
     * Returns the Controller object controlling the subsystem
     *
     * @return The Controller
     */
    public ControllerBase getController() {
        return controller;
    }

    /**
     * Sets the safetyThreshold to disable PID if setpoint is not reached
     * @param timeSeconds The time in seconds for the safety threshold
     */
    public void setSafetyThresh(double timeSeconds) {
        safetyThresh = timeSeconds;
    }

    /**
     * Called when the safety timeout is reached.
     * Disables the PID control.
     */
    public void onSafetyTimeout(){
        Log.unusual(getName(), "Safety Timeout Reached");
        disable();
    }

    /**
     * Sets the setpoint for the subsystem.
     *
     * @param setpoint the setpoint for the subsystem
     */
    public void goTo(double setpoint) {
        if (controller != null) {
            enable();
            controller.setSetpoint((debug != null && debug.getAsBoolean()) ? this.setpoint.getAsDouble() : setpoint);
        } else {
            runVolts((debug != null && debug.getAsBoolean()) ? this.setpoint.getAsDouble() : setpoint);
        }
    }
        

    public Command goToCommand(double setpoint) {
        return runOnce(() -> goTo(setpoint));
    }

    /**
     * Returns the current setpoint of the subsystem.
     *
     * @return The current setpoint
     */
    public double getSetpoint() {
        return controller.getSetpoint();
    }

    /**
     * Returns true if subsystem is at setpoint, false if not
     *
     * @return If subsystem is at setpoint
     */
    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    /** Enables the PID control. Resets the controller. */
    public void enable() {
        controller.enable();
        safetyTimer.restart();
        controller.reset();
        Log.debug(Log.Type.CONTROLLER, getName(), "Enabled PID");
    }

    /** Disables the PID control. Sets output to zero. */
    public void disable() {
        controller.disable();
        Log.debug(Log.Type.CONTROLLER, getName(), "Disabled PID");
    }

    /**
     * Returns whether the controller is enabled.
     *
     * @return Whether the controller is enabled.
     */
    public boolean isEnabled() {
        return controller.isEnabled();
    }

    /**
     * Sets power to motors.
     * 
     * @param power The power to set the motors to between -1 and 1.
     */
    public void run(double power) {
        for (NAR_Motor motor : motors) {
            motor.set(power);
        }
    }

    /**
     * Sets power to motors.
     * 
     * @param power The power to set the motors to between -1 and 1.
     * @return Command to run power
     */
    public Command runCommand(double power) {
        return runOnce(() -> run(power));
    }

    /**
     * Sets voltage to motors.
     * 
     * @param volts The voltage to set the motors to.
     */
    public void runVolts(double volts) {
        for (NAR_Motor motor : motors) {
            motor.setVolts(volts);
        }
    }

    /**
     * Sets voltage to motors.
     * 
     * @param volts The voltage to set the motors to.
     */
    public Command runVoltsCommand(double volts) {
        return runOnce(() -> runVolts(volts));
    }

    /**
     * Stops all motors.
     */
    public void stop() {
        disable();
        run(0);
    }

    /**
     * Resets measurement position to controller position minimum.
     */
    public void reset() {
        for (NAR_Motor motor : motors) {
            motor.resetPosition(((PositionController) controller).getInputRange()[0]);
        }
    }

    /**
     * Resets measurement position to controller position minimum.
     */
    public Command resetCommand() {
        return runOnce(() -> reset());
    }

    /**
     * Get the position of the mechanism relative to its reset.
     * 
     * @return The position of the first motor.
     */
    public double getPosition() {
        return motors[0].getPosition();
    }

    /**
     * Get the velocity of the mechanism.
     * 
     * @return The velocity of the first motor.
     */
    public double getVelocity() {
        return motors[0].getVelocity();
    }

    /**
     * Get the volts applied to the mechanism
     * 
     * @return The volts applied to the first motor.
     */
    public double getVolts() {
        return motors[0].getAppliedOutput() * 12;
    }

    public Command characterization(double rampRate, double stepVoltage) {
       NAR_SysIdCommand characterize = new NAR_SysIdCommand(rampRate, stepVoltage, (v) -> runVolts(v.in(Volts)), this, motors[0]);
       return characterize.runSysId();
    }

    public void initShuffleboard() {
        NAR_Shuffleboard.addData(getName(), "Enabled", this::isEnabled, 0, 0);
        NAR_Shuffleboard.addData(getName(), "AtSetpoint", this::atSetpoint, 1, 0);
        NAR_Shuffleboard.addData(getName(), "Measurement", controller != null ? controller::getMeasurement : this::getVolts, 0, 1);
        NAR_Shuffleboard.addData(getName(), "Setpoint", this::getSetpoint, 1, 1);

        debug = NAR_Shuffleboard.debugSwitch(getName(), "DEBUG", false, 2, 0);
        setpoint = NAR_Shuffleboard.debug(getName(), "Debug_Setpoint", 0, 2,1);

        NAR_Shuffleboard.addData(getName(), "Measurement Graph", controller != null ? controller::getMeasurement : this::getVolts, 6, 0, 2, 2).withWidget(BuiltInWidgets.kGraph);
        NAR_Shuffleboard.addData(getName(), "Setpoint Graph", this::getSetpoint, 8, 0, 2, 2).withWidget(BuiltInWidgets.kGraph);

        if (controller != null) {
            NAR_Shuffleboard.addSendable(getName(), "PID_Controller", controller, 3, 1, 2, 3).withWidget(BuiltInWidgets.kPIDController);
            FFWidgets(controller,1,0);
            runVoltsWidgets("running", debug, 1, 0);
        }

        NAR_Shuffleboard.addCommand(getName(), "Enable", either(startEnd(()-> goTo(setpoint.getAsDouble()), ()-> disable()), print("DEBUG NOT ON"), debug), 4, 0);
        NAR_Shuffleboard.addCommand(getName(), "Reset", either(resetCommand(), print("DEBUG NOT ON"), debug), 3, 0);
    }

    private void runVoltsWidgets(String tab, BooleanSupplier debug, int x, int y) {
        final DoubleSupplier debugVoltage = NAR_Shuffleboard.debug(getName(), "Debug Volts", 0, x+6, y + 3);
        NAR_Shuffleboard.addCommand(getName(), "Run Volts", either(startEnd(()-> runVolts(debugVoltage.getAsDouble()), ()-> stop()), print("DEBUG NOT ON"), debug), x+7, y+3);
        NAR_Shuffleboard.addData(getName(), "Running", () -> debug.getAsBoolean() && getVolts() > 0, x + 5, y+3);
        NAR_Shuffleboard.addData(getName(), "Voltage", this::getVolts, x + 4, y + 3);
    }

    private void FFWidgets(ControllerBase controller, int x, int y) {
        controller.getConfig().setkS(NAR_Shuffleboard.debug(getName(), "kS", controller.getConfig().getkS(), x, y+2));
        controller.getConfig().setkV(NAR_Shuffleboard.debug(getName(), "kV", controller.getConfig().getkV(), x + 1, y+2));
        controller.getConfig().setkA(NAR_Shuffleboard.debug(getName(), "kA", controller.getConfig().getkA(), x + 1, y + 3));
        controller.getConfig().setkG(NAR_Shuffleboard.debug(getName(), "kG", controller.getConfig().getkG(), x, y + 3));
        NAR_Shuffleboard.addCommand(getName(), "Characterize", this.characterization(1, 0.5), x-1, y+3).withSize(1, 1);
    }

}