package common.utility.tester;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import common.hardware.motorcontroller.NAR_Motor;
import common.utility.tester.Tester.SystemsTest;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BooleanSupplier;

/**
 * SystemsTests specifically for checking stall current.
 */
public class CurrentTest extends SystemsTest {

    private NAR_Motor motor;
    private double plateau;
    private boolean hasDelayed;
    private Timer timer;

    /**
     * Creates a current test.
     * @param testName Name of the test.
     * @param motor Motor being run.
     * @param power Power to run the motor at.
     * @param timeout Time the test is run for.
     * @param plateau How long the pass condition must be true for.
     * @param expectedCurrent Expected current drawn from test.
     * @param tolerance Max error of actual minus expected current.
     * @param subsystems Subsystems being used.
     */
    public CurrentTest(
            String testName, 
            NAR_Motor motor, 
            double power, 
            double timeout, 
            double plateau, 
            double expectedCurrent, 
            double tolerance, 
            Subsystem... subsystems) {
        this (
            testName,
            motor,
            power,
            timeout,
            plateau,
            ()-> Math.abs(Math.abs(motor.getStallCurrent()) - expectedCurrent) < tolerance
        );
    }

    /**
     * Creates a current test.
     * @param testName Name of the test.
     * @param motor Motor being run.
     * @param power Power to run the motor at.
     * @param timeout Time the test is run for.
     * @param plateau How long the pass condition must be true for.
     * @param peakCurrent The stall current the motor has to go above. 
     * @param subsystems Subsystems being used.
     */
    public CurrentTest(
            String testName, 
            NAR_Motor motor, 
            double power, 
            double timeout, 
            double plateau, 
            double peakCurrent, 
            Subsystem... subsystems) {
        this (
            testName,
            motor,
            power,
            timeout,
            plateau,
            ()-> Math.abs(motor.getStallCurrent()) > peakCurrent
        );
    }

    /**
     * Creates a current test.
     * @param testName Name of the test.
     * @param motor Motor being run.
     * @param power Power to run the motor at.
     * @param timeout Time the test is run for.
     * @param plateau How long the pass condition must be true for.
     * @param passCondition Condition for the test to pass.
     * @param subsystems Subsystems being used.
     */
    public CurrentTest(
            String testName, 
            NAR_Motor motor, 
            double power, 
            double timeout, 
            double plateau, 
            BooleanSupplier passCondition, 
            Subsystem... subsystems) {
        super(
            testName, 
            runOnce(()-> motor.set(power), subsystems).andThen(waitSeconds(timeout)),
            passCondition
        );
        this.motor = motor;
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
        motor.set(0);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || (timer.hasElapsed(plateau) && hasDelayed);
    }
}
