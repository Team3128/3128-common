package common.utility.tester;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import common.utility.tester.Tester.UnitTest;
import edu.wpi.first.wpilibj.Timer;

public class Plateau extends UnitTest {
    private double plateau;
    private Timer timer;
    private boolean hasDelayed;
    public static double timeout = 30;

    /**
     * 
     * @param testName Name of test.
     * @param plateau How long the pass condition must be true for.
     * @param measurement Source of measurement
     * @param expected Expected value
     * @param tolerance Maximum error.
     */
    public Plateau(String testName, double plateau, DoubleSupplier measurement, double expected, double tolerance) {
        this(testName, plateau, () -> Math.abs(measurement.getAsDouble() - expected), tolerance, () -> (Math.abs(measurement.getAsDouble() - expected) < tolerance));
    }

    /**
     * 
     * @param testName Name of test.
     * @param plateau How long the pass condition must be true for.
     * @param error Difference from expected value.
     * @param tolerance Maximum error.
     */
    public Plateau(String testName, double plateau, DoubleSupplier error, double tolerance) {
        this(testName, plateau, error, tolerance, () -> Math.abs(error.getAsDouble()) < tolerance);
    }

    /**
     * 
     * @param testName Name of test.
     * @param plateau How long the pass condition must be true for.
     * @param error Difference from expected value.
     * @param tolerance Maximum error.
     * @param passCondition Condition for the test to pass.
     */
    public Plateau(String testName, double plateau, DoubleSupplier error, double tolerance, BooleanSupplier passCondition) {
        super(
            testName,
            race(run(()-> error.getAsDouble()), waitSeconds(timeout)),
            passCondition
        );
        this.plateau = plateau;
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
        return super.isFinished() || (timer.hasElapsed(plateau) && hasDelayed);
    }
    
    
}
