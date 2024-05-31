package common.utility.tester;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import common.utility.tester.Tester.UnitTest;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class PlateauTest extends UnitTest {
    private double plateau;
    private Timer timer;
    private boolean hasDelayed;
    public static double timeout = 30;

    /**
     * Creates a plateau test.
     * @param testName Name of test.
     * @param command Command to run.
     * @param plateau How long the pass condition must be true for.
     * @param error Difference from expected value.
     * @param tolerance Maximum error.
     * @param timeOut Time to run the test.
     */
    public PlateauTest(
            String testName,  
            Command command, 
            double plateau,
            DoubleSupplier error, 
            double tolerance, 
            double timeOut) {
        this(
            testName, 
            command, 
            plateau, 
            timeout,
            () -> Math.abs(error.getAsDouble()) < tolerance);
    }

    /**
     * Creates a plateau test.
     * @param testName Name of test.
     * @param command Command to run.
     * @param plateau How long the pass condition must be true for.
     * @param timeOut Time to run the test.
     * @param passCondition Condition to pass the test.
     */
    public PlateauTest(
            String testName,  
            Command command, 
            double plateau, 
            double timeOut, 
            BooleanSupplier passCondition) {
        super(
            testName,
            command.andThen(waitSeconds(timeOut)),
            passCondition
        );
        this.plateau = plateau;
        timer = new Timer();
        hasDelayed = false;
    }

    /**
     * Creates a plateau test.
     * @param testName Name of test.
     * @param command Command to run.
     * @param plateau How long the pass condition must be true for.
     * @param error Difference from expected value.
     * @param tolerance Maximum error.
     */
    public PlateauTest(
            String testName,  
            Command command, 
            double plateau, 
            DoubleSupplier error, 
            double tolerance) {
        this(
            testName, 
            command, 
            plateau, 
            () -> Math.abs(error.getAsDouble()) < tolerance);
    }

    /**
     * Creates a plateau test.
     * @param testName Name of test.
     * @param command Command to run.
     * @param plateau How long the pass condition must be true for.
     * @param passCondition Condition for the test to pass.
     */
    public PlateauTest(
            String testName,  
            Command command, 
            double plateau, 
            BooleanSupplier passCondition) {
        super(
            testName,
            command.andThen(waitSeconds(timeout)),
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
