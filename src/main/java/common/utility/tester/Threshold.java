package common.utility.tester;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import common.hardware.motorcontroller.NAR_Motor;
import common.utility.Log;
import common.utility.tester.Tester.TestState;
import common.utility.tester.Tester.UnitTest;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Threshold extends UnitTest {
    public String testName;
    private double plateau; //not sure if this is necessary. do we just want it to reach the threshold or does it have to HOLD it?
    private boolean hasDelayed; //sometimes, i assume, the system will delay for one second; so we have to take that into account just in case it happens?
    private Timer timer;
    private static double timeout = 30;

    /**
     * Creates a Threshold test.
     * @param testName Name of the test.
     * @param measurement Source of measurement.
     * @param threshold Threshold that should be reached.
     */
    public Threshold(String testName, DoubleSupplier measurement, double threshold) {
        this(testName, measurement, ()-> measurement.getAsDouble() > threshold);
    }


    /**
     * Creates a current test.
     * @param testName Name of the test.
     * @param measurement Source of measurement.
     * @param threshold Threshold that should be reached.
     * @param passCondition Condition for the test to pass.
     */
    public Threshold(
            String testName, 
            DoubleSupplier measurement,
            BooleanSupplier passCondition) {
        super(
            testName, 
            race(run(() -> measurement.getAsDouble()), waitSeconds(timeout)),
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
