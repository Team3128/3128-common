package common.core.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import common.core.controllers.ControllerBase;
import common.hardware.motorcontroller.NAR_Motor;
import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;
import common.utility.tester.Tester;
import common.utility.tester.Tester.SystemsTest;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem based off of {@link PIDSubsystem} 
 * @since 2023 Charged Up
 * @author Mason Lam
 */
public abstract class NAR_PIDSubsystem extends SubsystemBase {

    /**
     * SystemsTest specifically for PIDSubsystems.
     */
    public class SetpointTest extends SystemsTest {
        private final double timeOut;
        private final Timer timer = new Timer();
        private double prevTime;

        /**
         * Creates a setpoint test for the system.
         * @param testName Name of the test.
         * @param setpoint Setpoint for the system to try and reach.
         * @param plateau Time the subsystem has to hold the setpoint.
         * @param timeOut The time the system has to reach the setpoint.
         */
        public SetpointTest(String testName, double setpoint, double plateau, double timeOut) {
            super(testName, runOnce(()-> startPID(setpoint)).andThen(waitSeconds(timeOut)));
            this.timeOut = timeOut;
            passCondition = ()-> timer.hasElapsed(plateau);
            prevTime = 0;
        }

        @Override
        public void initialize() {
            super.initialize();
            timer.restart();
            prevTime = Timer.getFPGATimestamp();
        }

        @Override
        public void execute() {
            super.execute();
            if (!atSetpoint()) timer.reset();
        }

        @Override
        public void end(boolean interrupted) {
            super.end(interrupted);
            Log.info(testName, "Expected Time: " + timeOut);
            Log.info(testName, "Actual Time: " + (Timer.getFPGATimestamp() - prevTime));
        }

        @Override
        public boolean isFinished() {
            return super.isFinished() || passCondition.getAsBoolean();
        }

        /**
         * Add the test to the a Subsystem test
         */
        public void add() {
            Tester.getInstance().addTest(getName(), this);
        }
    }

    protected final ControllerBase controller;
    protected BooleanSupplier debug;
    protected DoubleSupplier setpoint;
    private double safetyThresh;
    private Timer safetyTimer = new Timer();

    private double prevMeasurement;
    private double prevVelocity;
    private double updateTime;
    private Timer updateTimer = new Timer();

    private List<BooleanSupplier> disableConditions = new ArrayList<>();

    private boolean shouldLog = false;

    /**
     * Creates a new PIDSubsystem.
     *
     * @param controller the Controller to use
     */
    public NAR_PIDSubsystem(ControllerBase controller) {
        this.controller = controller;
        safetyThresh = 5;
        prevMeasurement = 0;
        prevVelocity = 0;
        updateTime = controller.getPeriod();
        updateTimer.start();
    }

    public NAR_PIDSubsystem(ControllerBase controller, DoubleSupplier measurement, DoubleConsumer useOutput) {
        this(controller);
        controller.setMeasurementSource(measurement);
        controller.addOutput(useOutput);
    }

    public NAR_PIDSubsystem(ControllerBase controller, NAR_Motor motor) {
        this(controller);
        controller.configureFeedback(motor);
    }



    @Override
    public void periodic() {
        if (controller.isEnabled()) {
            controller.useOutput();
            if (safetyTimer.hasElapsed(safetyThresh)) onSafetyTimeout();
            if (atSetpoint()) safetyTimer.restart();
            if(controller.getMeasurement() < getInputRange()[0] || controller.getMeasurement() > getInputRange()[1]) onConstraintViolation();
            for (BooleanSupplier condition : disableConditions) {
                if (condition.getAsBoolean()) {
                    Log.info(getName(), "Disable Condition has been triggered.");
                    disable();
                    break;
                }
            }
        }

        if (!shouldLog) return;

        if (updateTimer.hasElapsed(updateTime)) {
            final double measurement = controller.getMeasurement();
            final double velocity = (measurement - prevMeasurement) / updateTimer.get();
            final double acceleration = (velocity - prevVelocity) / updateTimer.get();
            NAR_Shuffleboard.addData(getName(), "1stDerivative", velocity, 1, 2);
            NAR_Shuffleboard.addData(getName(), "2ndDerivative", acceleration, 1, 3);
            prevMeasurement = measurement;
            prevVelocity = velocity;
            updateTimer.restart();
        }
    }
    /**
     * Initializes shuffleboard with debug elements for PID + FF values.
     */
    public void initShuffleboard() {
        shouldLog = true;
        NAR_Shuffleboard.addSendable(getName(), "PID_Controller", controller, 0, 0);
        NAR_Shuffleboard.addData(getName(), "Setpoint", ()-> getSetpoint(), 0, 2);
        NAR_Shuffleboard.addData(getName(), "AtSetpoint", ()-> atSetpoint(), 0, 3);

        NAR_Shuffleboard.addData(getName(), "Enabled", ()-> isEnabled(), 1, 0);
        NAR_Shuffleboard.addData(getName(), "Measurement", ()-> controller.getMeasurement(), 1, 1);

        NAR_Shuffleboard.addData(getName(), "TOGGLE", false, 2, 0).withWidget("Toggle Button");
        debug = NAR_Shuffleboard.getBoolean(getName(), "TOGGLE");
        NAR_Shuffleboard.addData(getName(), "DEBUG", ()-> debug.getAsBoolean(), 2, 1);
        setpoint = NAR_Shuffleboard.debug(getName(), "Debug_Setpoint", 0, 2,2);
        NAR_Shuffleboard.addData(getName(), "Output", ()-> controller.useOutput(), 2, 3);

        controller.getConfig().setkS(NAR_Shuffleboard.debug(getName(), "kS", controller.getConfig().getkS(), 3, 0));
        controller.getConfig().setkV(NAR_Shuffleboard.debug(getName(), "kV", controller.getConfig().getkV(), 3, 1));
        controller.getConfig().setkA(NAR_Shuffleboard.debug(getName(), "kA", controller.getConfig().getkA(), 3, 2));
        controller.getConfig().setkG(NAR_Shuffleboard.debug(getName(), "kG", controller.getConfig().getkG(), 3, 3));
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
     * Sets the amount of time between measurement logging to account for noisy measurements.
     * @param timeSeconds The time in seconds between each update.
     */
    public void setUpdateTime(double timeSeconds) {
        updateTime = timeSeconds;
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
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     */
    public void setTolerance(double positionTolerance) {
        controller.setTolerance(positionTolerance);
    }

    /**
     * Adds a condition to disable the PID control.
     * @param condition The condition to disable the PID control.
     */
    public void addDisableCondition(BooleanSupplier condition) {
        disableConditions.add(condition);
    }

    /**
     * Returns the list of disable conditions
     * @return List of disable conditions
     */
    public List<BooleanSupplier> getDisableConditions() {
        return disableConditions;
    }

    /**
     * Sets the minimum and maximum values expected from the input.
     *
     * <p>Input values outside of this range will be clamped.
     *
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    public void setInputRange(double minimumInput, double maximumInput) {
        controller.setInputRange(minimumInput, maximumInput);
    }

    /**
     * Returns the minimum and maximum values expected from the input.
     * 
     * <p>First element is inputMin.
     * <p>Second element is inputMax.
     * 
     * @return double[] with inputMin and inputMax.
     */
    public double[] getInputRange() {
        return controller.getInputRange();
    }

    /**
     * Sets the minimum and maximum values of the controller output.
     *
     * <p>Output values outside of this range will be clamped.
     *
     * @param minimumOutput The minimum value to write.
     * @param maximumOutput The maximum value to write.
     */
    public void setOutputRange(double minimumOutput, double maximumOutput) {
        controller.setOutputRange(minimumOutput, maximumOutput);
    }

    /**
     * Returns the minimum and maximum values expected from the output.
     * 
     * <p>First element is outputMin.
     * <p>Second element is outputMax.
     * 
     * @return double[] with outputMin and outputMax.
     */
    public double[] getOutputRange() {
        return controller.getOutputRange();
    }

    /**
     * Called when the measurement of the controller violates the constraints
     */
    public void onConstraintViolation() {
        Log.unusual(getName(), "Controller Measurement Violated Input Constraints");
    }

    /**
     * Sets the function returning the value multiplied against kG
     * @param kG_Function the function multiplied to kG
     */
    public void setkG_Function(DoubleSupplier kG_Function) {
        controller.getConfig().setkG_Function(kG_Function);
    }

    /**
     * Sets the setpoint for the subsystem.
     *
     * @param setpoint the setpoint for the subsystem
     */
    public void startPID(double setpoint) {
        enable();
        controller.setSetpoint((debug != null && debug.getAsBoolean()) ? this.setpoint.getAsDouble() : setpoint);
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
        controller.enableContinuousInput(minimumInput, maximumInput);
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

    // /**
    //  * Uses the output from the PIDController.
    //  *
    //  * @param output The output of the PIDController
    //  * @param setpoint The setpoint of the controller.
    //  */
    // protected abstract void useOutput(double output, double setpoint);

    // /**
    //  * Returns the measurement of the process variable used by the PIDController.
    //  *
    //  * @return the measurement of the process variable
    //  */
    // protected abstract double getMeasurement();

    /** Enables the PID control. Resets the controller. */
    public void enable() {
        controller.enable();
        safetyTimer.restart();
        controller.reset();
    }

    /** Disables the PID control. Sets output to zero. */
    public void disable() {
        controller.disable();
        // useOutput(0, 0);
        Log.info(getName(), "Disabled");
    }

    /**
     * Return whether the subsystem is in debug mode.
     * @return Boolean, true being in debug, false being not in debug.
     */
    public boolean isDebug() {
        return debug.getAsBoolean();
    }

    /**
     * Returns whether the controller is enabled.
     *
     * @return Whether the controller is enabled.
     */
    public boolean isEnabled() {
        return controller.isEnabled();
    }
}