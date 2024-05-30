package common.utility.tester;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import common.utility.tester.Tester.UnitTest;
import edu.wpi.first.wpilibj.Timer;

public class Time extends UnitTest {
    private double plateau;
    private Timer timer;
    private boolean hasDelayed;
    
    /**
     * 
     * @param testName Name of the test.
     * @param plateau How long the pass condition must be held.
     * @param measurement Actual value.
     * @param expected Expected value.
     * @param tolerance Maximum error.
     * @param timeout Time the test is run for.
     */
    public Time(String testName, double plateau, DoubleSupplier measurement, double expected, double tolerance, double timeout) {
        this(testName, plateau, () -> Math.abs(measurement.getAsDouble() - expected), tolerance, timeout, () -> (Math.abs(measurement.getAsDouble() - expected) < tolerance));
    }

    /**
     * 
     * @param testName Name of the test.
     * @param plateau How long the pass condition must be held.
     * @param error Difference from expected value.
     * @param tolerance Maximum error.
     * @param timeout Time the test is run for.
     */
    public Time(String testName, double plateau, DoubleSupplier error, double tolerance, double timeout) {
        this(testName, plateau, error, tolerance, timeout, () -> (Math.abs(error.getAsDouble()) < tolerance));
    }

    /**
     * 
     * @param testName Name of the test.
     * @param plateau How long the pass condition must be held.
     * @param error Difference from expected value.
     * @param tolerance Maximum error.
     * @param timeout Time the test is run for.
     * @param passCondition Condition for the test to pass.
     */
    public Time(String testName, double plateau, DoubleSupplier error, double tolerance, double timeout,
            BooleanSupplier passCondition) {
        super(
            testName,
            race(run(() -> error.getAsDouble()), waitSeconds(timeout)),
            passCondition
        );
        this.plateau = plateau;
        timer = new Timer();
        hasDelayed = false;
    }

    @Override
    public void initialize(){
        super.initialize();
        timer.restart();
        hasDelayed = false;
    }

    @Override
    public void execute(){
        super.execute();
        if(!hasDelayed){
            if(timer.hasElapsed(1)){
                hasDelayed = true;
                timer.reset();
            }
            return;
        }
        if (!passCondition.getAsBoolean()) timer.reset();
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
    }

    @Override
    public boolean isFinished(){
        return super.isFinished() || (timer.hasElapsed(plateau) && !hasDelayed);
    }


}
