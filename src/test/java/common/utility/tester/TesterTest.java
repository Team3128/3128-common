package common.utility.tester;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import common.utility.tester.Tester.SystemsTest;
import common.utility.tester.Tester.TestState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Covers {@link SystemsTest}. {@code Tester.Test} isn't covered, since creating one starts
 * NarwhalDashboard's web server.
 */
class TesterTest {

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    /** A command that counts how it's run. */
    private static class CountingCommand extends Command {
        int initialized, executed, ended;
        boolean endedInterrupted;

        @Override
        public void initialize() {
            initialized++;
        }

        @Override
        public void execute() {
            executed++;
        }

        @Override
        public void end(boolean interrupted) {
            ended++;
            endedInterrupted = interrupted;
        }
    }

    @Test
    void testPassesWhenItFinishesAndTheConditionHolds() {
        final SystemsTest test = new SystemsTest("Intake", Commands.none(), () -> true);

        test.initialize();
        assertEquals(TestState.RUNNING, test.testState);
        test.end(false);

        assertEquals(TestState.PASSED, test.testState);
    }

    @Test
    void testFailsWhenTheConditionDoesNotHold() {
        final SystemsTest test = new SystemsTest("Intake", Commands.none(), () -> false);
        test.initialize();
        test.end(false);
        assertEquals(TestState.FAILED, test.testState);
    }

    @Test
    void testFailsWhenInterrupted() {
        final SystemsTest test = new SystemsTest("Intake", Commands.none());
        test.initialize();
        test.end(true);
        assertEquals(TestState.FAILED, test.testState);
    }

    @Test
    void testRunsTheWrappedCommand() {
        final CountingCommand command = new CountingCommand();
        final SystemsTest test = new SystemsTest("Intake", command);

        test.initialize();
        test.execute();
        test.execute();
        test.end(true);

        assertEquals(1, command.initialized);
        assertEquals(2, command.executed);
        assertEquals(1, command.ended);
        assertTrue(command.endedInterrupted);
    }

    @Test
    void testNeedsTheSameSubsystemsAsItsCommand() {
        final SubsystemBase intake = new SubsystemBase() {};
        final SystemsTest test = new SystemsTest("Intake", Commands.runOnce(() -> {}, intake));
        assertTrue(test.getRequirements().contains(intake));
    }
}
