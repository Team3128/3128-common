package common.utility.tester;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import common.utility.tester.Tester.UnitTest;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ThresholdTest extends UnitTest {
    public String testName;
    private boolean hasDelayed; //sometimes, i assume, the system will delay for one second; so we have to take that into account just in case it happens?
    private Timer timer;
    private static double timeout = 30;

        /**
     * Creates a threshold test.
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
            ()-> measurement.getAsDouble() > threshold);
    }


    /**
     * Creates a threshold test.
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
        super(
            testName, 
            command.andThen(waitSeconds(timeOut)),
            passCondition
        );
        timer = new Timer();
        hasDelayed = false;
    }

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
        this(testName, command, ()-> measurement.getAsDouble() > threshold);
    }


    /**
     * Creates a threshold test.
     * @param testName Name of the test.
     * @param command Command to run.
     * @param passCondition Condition for the test to pass.
     */
    public ThresholdTest(
            String testName, 
            Command command,
            BooleanSupplier passCondition) {
        super(
            testName, 
            command.andThen(waitSeconds(timeout)),
            passCondition
        );
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
            if (timer.hasElapsed(1)) {
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
        return super.isFinished() || (hasDelayed);
    }
    
}
