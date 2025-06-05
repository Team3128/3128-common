package common.utility.tester;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import common.utility.tester.Tester.SystemsTest;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class PlateauTest extends SystemsTest {
    private double plateau;
    private Timer timer;

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
            error,
            tolerance,
            30);
    }

    /**
     * Creates a plateau test. (specify timeout)
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
            timeOut,
            () -> Math.abs(error.getAsDouble()) < tolerance);
    }

    /**
     * Creates a plateau test. (specify passCondition)
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
        this(
            testName,
            command, 
            plateau, 
            30,
            passCondition);
        this.plateau = plateau;
        timer = new Timer();
    }

    /**
     * Creates a plateau test. (specify passCondition and timeout)
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
            command.withTimeout(timeOut),
            passCondition
        );
        this.plateau = plateau;
        timer = new Timer();
    }


    @Override
    public void initialize() {
        super.initialize();
        timer.restart();
    }

    @Override
    public void execute() {
        super.execute();
        if (!passCondition.getAsBoolean()) timer.reset();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || (timer.hasElapsed(plateau) && passCondition.getAsBoolean());
    }
    
    
}
