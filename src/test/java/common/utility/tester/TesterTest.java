package common.utility.tester;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import common.utility.tester.Tester.SystemsTest;
import common.utility.tester.Tester.TestState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Covers single checks ({@link SystemsTest}) and groups of them ({@code Tester.Test}). Making a
 * group starts NarwhalDashboard's server, which is harmless in a test.
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

    @BeforeEach
    void enableRobot() {
        // Only this test's commands, not subsystems other test classes left registered
        CommandScheduler.getInstance().unregisterAllSubsystems();
        // The scheduler ignores commands scheduled while the robot is disabled
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    @AfterEach
    void disableRobot() {
        CommandScheduler.getInstance().cancelAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    /**
     * Runs the command scheduler long enough for {@code test} to finish. A group counts as
     * finished before it starts, so this runs a fixed number of loops.
     */
    private static void runUntilDone(Tester.Test test) {
        for (int i = 0; i < 20; i++) {
            CommandScheduler.getInstance().run();
        }
    }

    @Test
    void groupRunsEachCheckInOrderAndPasses() {
        final Tester tester = Tester.getInstance();
        final List<String> ran = new ArrayList<>();
        tester.addTest("Intake group", new SystemsTest("Extend", Commands.runOnce(() -> ran.add("Extend"))));
        tester.addTest("Intake group", new SystemsTest("Spin", Commands.runOnce(() -> ran.add("Spin"))));
        final Tester.Test group = tester.getTest("Intake group");
        group.setTimeBetweenTests(0);

        tester.runTest("Intake group");
        runUntilDone(group);

        assertEquals(List.of("Extend", "Spin"), ran);
        assertEquals(TestState.PASSED, group.getTestState());
    }

    @Test
    void groupStopsAtTheFirstFailure() {
        final Tester tester = Tester.getInstance();
        final List<String> ran = new ArrayList<>();
        tester.addTest("Shooter group", new SystemsTest("Spin up", Commands.runOnce(() -> ran.add("Spin up")), () -> false));
        tester.addTest("Shooter group", new SystemsTest("Shoot", Commands.runOnce(() -> ran.add("Shoot"))));
        final Tester.Test group = tester.getTest("Shooter group");

        CommandScheduler.getInstance().schedule(group);
        runUntilDone(group);

        assertEquals(List.of("Spin up"), ran);
        assertEquals(TestState.FAILED, group.getTestState());
    }

    @Test
    void groupsCanContainOtherGroups() {
        final Tester tester = Tester.getInstance();
        tester.addTest("Hopper group", new SystemsTest("Feed", Commands.none()));
        tester.addTest("Whole robot", tester.getTest("Hopper group"));
        final Tester.Test robot = tester.getTest("Whole robot");

        CommandScheduler.getInstance().schedule(robot);
        runUntilDone(robot);

        assertEquals(TestState.PASSED, robot.getTestState());
    }

    @Test
    void testNeedsTheSameSubsystemsAsItsCommand() {
        final SubsystemBase intake = new SubsystemBase() {};
        final SystemsTest test = new SystemsTest("Intake", Commands.runOnce(() -> {}, intake));
        assertTrue(test.getRequirements().contains(intake));
    }
}
