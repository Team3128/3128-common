package common.core.subsystems;

import java.util.HashSet;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import common.core.controllers.Controller;
import common.core.controllers.Controller.Type;
import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj2.command.Commands.*;

/**
 * A subsystem based off of {@link PIDSubsystem} 
 * @since 2023 Charged Up
 * @author Mason Lam
 */
public abstract class NAR_PIDSubsystem extends SubsystemBase {

    public class Test {
        public double setpoint;
        public double time;
        public boolean passedTest;
        private double startTime;
        private double endTime;

        /**
         * Creates a new Test for the PIDSubsystem
         * @param setpoint The setpoint for the test goes to
         * @param time The time for the subsystem to reach setpoint
         */
        public Test(double setpoint, double time) {
            this.setpoint = setpoint;
            this.time = time;
            passedTest = false;
            startTime = 0;
            endTime = 0;
        }

        private void check() {
            passedTest = time >= (endTime - startTime);
        }
    }

    private HashSet<Test> unitTests = new HashSet<Test>();

    protected final Controller m_controller;
    protected boolean m_enabled;
    private BooleanSupplier debug;
    private DoubleSupplier setpoint;
    private double min, max;
    private double safetyThresh;
    private double plateauCount;

    /**
     * Creates a new PIDSubsystem.
     *
     * @param controller the Controller to use
     */
    public NAR_PIDSubsystem(Controller controller) {
        m_controller = controller;
        controller.setMeasurementSource(()-> getMeasurement());
        controller.addOutput(output -> useOutput(output, getSetpoint()));
        min = Double.NEGATIVE_INFINITY;
        max = Double.POSITIVE_INFINITY;
        safetyThresh = 5;
        plateauCount = 0;
        NarwhalDashboard.getInstance().addUpdate(this.getName(), ()-> hasPassedTest());
    }

    @Override
    public void periodic() {
        if (m_enabled) {
            m_controller.useOutput();
            if (plateauCount * 0.02 > safetyThresh) disable();
            if (atSetpoint()) plateauCount = 0;
            else plateauCount ++;
        }
    }

    /**
     * Initializes shuffleboard with debug elements for PID + FF values.
     */
    public void initShuffleboard() {
        NAR_Shuffleboard.addComplex(getName(), "PID_Controller", m_controller, 0, 0);

        NAR_Shuffleboard.addData(getName(), "Enabled", ()-> isEnabled(), 1, 0);
        NAR_Shuffleboard.addData(getName(), "Measurement", ()-> getMeasurement(), 1, 1);
        NAR_Shuffleboard.addData(getName(), "Setpoint", ()-> getSetpoint(), 1, 2);

        var debugEntry = NAR_Shuffleboard.addData(getName(), "TOGGLE", false, 2, 0).withWidget("Toggle Button");
        debug = ()-> debugEntry.getEntry().getBoolean(false);
        NAR_Shuffleboard.addData(getName(), "DEBUG", ()-> debug.getAsBoolean(), 2, 1);
        setpoint = NAR_Shuffleboard.debug(getName(), "Debug_Setpoint", 0, 2,2);

        if (m_controller.getType() == Type.DEFAULT) return;

        m_controller.setkS(NAR_Shuffleboard.debug(getName(), "kS", m_controller.getkS(), 3, 0));

        if (m_controller.getType() == Type.VELOCITY) {
            m_controller.setkV(NAR_Shuffleboard.debug(getName(), "kV", m_controller.getkV(), 3, 1));
        }
        else {
            m_controller.setkG(NAR_Shuffleboard.debug(getName(), "kG", m_controller.getkG(), 3, 1));
        }
    }

    /**
     * Returns the Controller object controlling the subsystem
     *
     * @return The Controller
     */
    public Controller getController() {
        return m_controller;
    }

    /**
     * Sets the safetyThreshold to disable PID if setpoint is not reached
     * @param timeSeconds the time in seconds for the safety threshold
     */
    public void setSafetyThresh(double timeSeconds) {
        safetyThresh = timeSeconds;
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
     * Runs the list of unit tests given 
     * @param tests Tests to be run on the subsystem
     * @return Sequential Command Group containing all unit tests
     */
    public CommandBase runTest(Test... tests) {
        final CommandBase[] commands = new CommandBase[tests.length];
        for (int index = 0; index < tests.length; index ++) {
            commands[index] = runTest(tests[index]);
        }
        return sequence(commands);
    }

    /**
     * Runs the unit test
     * @param test Test for the subsystem
     * @return Command which will run the test
     */
    private CommandBase runTest(Test test) {
        unitTests.add(test);
        return sequence(
            runOnce(()-> test.passedTest = false),
            runOnce(()-> test.startTime = Timer.getFPGATimestamp()),
            runOnce(()-> startPID(test.setpoint)),
            waitUntil(()-> atSetpoint()),
            runOnce(()-> test.endTime = Timer.getFPGATimestamp()),
            runOnce(()-> test.check()),
            runOnce(()-> Log.info(this.getName(), "Expected Time: " + test.time + " Actual Time: " + (test.endTime - test.startTime))),
            either(print(this.getName() + " TEST PASSED"), print(this.getName() + " TEST FAILED"), ()-> test.passedTest)
        );
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
    protected abstract void useOutput(double output, double setpoint);

    /**
     * Returns the measurement of the process variable used by the PIDController.
     *
     * @return the measurement of the process variable
     */
    protected abstract double getMeasurement();

    /** Enables the PID control. Resets the controller. */
    public void enable() {
        m_enabled = true;
        plateauCount = 0;
        m_controller.reset();
    }

    /** Disables the PID control. Sets output to zero. */
    public void disable() {
        m_enabled = false;
        useOutput(0, 0);
    }

    /**
     * Returns whether or not the subsystem has passed the test
     * @return True if tests have been passed and false if tests have failed
     */
    public boolean hasPassedTest() {
        if (unitTests.size() == 0) return false;
        for (final Test test : unitTests) {
            if (!test.passedTest) return false;
        }
        return true;
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
