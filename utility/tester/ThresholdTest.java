package common.utility.tester;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import common.utility.tester.Tester.SystemsTest;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ThresholdTest extends SystemsTest {
    public String testName;
    private boolean hasDelayed;
    private double delayPeriod;
    private Timer timer;

    /**
     * Creates a threshold test.
     * @param testName Name of the test.
     * @param command Command to run.
     * @param measurement Source of measurement.
     * @param threshold Threshold that should be reached.
     */
    public ThresholdTest(
            String testName, 
            Command command, 
            DoubleSupplier measurement, 
            double threshold) {
        this(
            testName, 
            command, 
            measurement,
            threshold,
            30
        );
    }

    /**
     * Creates a threshold test. (specify timeout)
     * @param testName Name of the test.
     * @param command Command to run.
     * @param measurement Source of measurement.
     * @param threshold Threshold that should be reached.
     * @param timeOut Time for test to run.
     */
    public ThresholdTest(
            String testName, 
            Command command,
            DoubleSupplier measurement, 
            double threshold, 
            double timeOut) {
        this(
            testName, 
            command, 
            timeOut, 
            ()-> Math.abs(measurement.getAsDouble()) > threshold);
    }


    /**
     * Creates a threshold test. (specify passCondition)
     * @param testName Name of the test.
     * @param command Command to run.
     * @param passCondition Condition for the test to pass.
     */
    public ThresholdTest(
            String testName, 
            Command command,
            BooleanSupplier passCondition) {
        this(
            testName, 
            command,
            30,
            passCondition
        );
        timer = new Timer();
        hasDelayed = false;
    }

    /**
     * Creates a threshold test. (specify timeout and passCondition)
     * @param testName Name of the test.
     * @param command Command to run.
     * @param timeOut Time for test to run.
     * @param passCondition Condition for the test to pass.
     */
    public ThresholdTest(
            String testName, 
            Command command,
            double timeOut,
            BooleanSupplier passCondition) {
        this(
            testName, 
            command,
            timeOut,
            1,
            passCondition
        );
    }

    /**
     * Creates a threshold test. (specify timeout, passCondition, and delayPeriod)
     * @param testName Name of the test.
     * @param command Command to run.
     * @param timeOut Time for test to run.
     * @param delayPeriod Time of delay in seconds.
     * @param passCondition Condition for the test to pass.
     */
    public ThresholdTest(
            String testName, 
            Command command,
            double timeOut,
            double delayPeriod,
            BooleanSupplier passCondition) {
        super(
            testName, 
            command.withTimeout(timeOut),
            passCondition
        );
        this.delayPeriod = delayPeriod;
        timer = new Timer();
        hasDelayed = false;
    }

    @Override
    public void initialize() {
        super.initialize();
        timer.restart();
        hasDelayed = false;
    }

    @Override
    public void execute() {
        super.execute();
        if (!hasDelayed) {
            if (timer.hasElapsed(delayPeriod)) {
                hasDelayed = true;
                timer.reset();
            }
            return;
        }
        if (!passCondition.getAsBoolean()) timer.reset();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || (passCondition.getAsBoolean() && hasDelayed);
    }
    
}
