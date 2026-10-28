package common.utility.tester;

import java.util.function.BooleanSupplier;
import java.util.HashMap;
import java.util.ArrayList;

import common.utility.Log;
import common.utility.dashboard.Dashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Runs system checks in the pits: quick tests that each part of the robot works before a match.
 *
 * <p>A {@link SystemsTest} is one check: a command to run, and optionally a condition that must
 * be true when it finishes for it to pass. Tests added under the same name form a {@link Test},
 * which runs them in order and stops at the first failure. Each name gets a button and a
 * pass/fail value on the dashboard, under {@code Tests/} and {@code Test States/}.
 *
 * <pre>{@code
 * Tester.getInstance().addTest("Intake", new SystemsTest("Extend",
 *     intake.setStateCommand(INTAKE).andThen(waitSeconds(1)),
 *     () -> intake.slider.atSetpoint()));
 * }</pre>
 */
public class Tester {

    /**
     * One check: runs a command, and passes if it finishes without being interrupted and its pass
     * condition is true at the end.
     */
    public static class SystemsTest extends Command {

        protected String testName;
        protected Command command;
        protected BooleanSupplier passCondition;
        protected TestState testState;
        /**
         * Creates a new System Test.
         * @param testName Name of the test.
         * @param command Command to be run for the test.
         */
        public SystemsTest(String testName, Command command) {
            this(testName, command, ()-> true);
        }

        /**
         * Creates a new System Test.
         * @param testName Name of the test.
         * @param command Command to be run for the test.
         * @param passCondition Condition for the test to pass.
         */
        public SystemsTest(String testName, Command command, BooleanSupplier passCondition) {
            this.testName = testName;
            this.command = command;
            this.passCondition = passCondition;
            testState = TestState.FAILED;
            for (final Subsystem subsystem : command.getRequirements()) {
                addRequirements(subsystem);
            }
        }

        @Override
        public void initialize() {
            Log.info(testName, "Test Running");
            command.initialize();
            testState = TestState.RUNNING;
        }

        @Override
        public void execute() {
            command.execute();
        }

        @Override
        public void end(boolean interrupted) {
            Log.info(testName, "Test Ended");
            testState = (interrupted || !passCondition.getAsBoolean()) ? TestState.FAILED : TestState.PASSED;
            Log.info(testName, "Test " + testState);
            command.end(interrupted);
        }

        @Override
        public boolean isFinished() {
            return command.isFinished();
        }
    }

    /**
     * A named group of checks for one part of the robot, run one after another. It stops at the
     * first failure, and passes once every check has passed.
     */
    public static class Test extends Command {
        private final ArrayList<SystemsTest> systemsTests;
        private final String name;
        private final Timer passTimer;
        private double timeBetweenTests;
        private SystemsTest testToSchedule;
        private TestState state;
        private int curIndex;

        /**
         * Creates a test for a specific system.
         * @param name Name of the test or system.
         */
        private Test(String name) {
            systemsTests = new ArrayList<SystemsTest>();
            this.name = name;
            this.timeBetweenTests = 0;
            state = TestState.FAILED;
            curIndex = 0;
            passTimer = new Timer();
            Dashboard.add("Tests/" + name, this);
            Dashboard.add("Test States/" + name, ()-> state.toString());
        }

        /**
         * Adds a larger system test to be run.
         * @param test Test for the robot.
         */
        public void addTest(Test test) {
            systemsTests.add(new SystemsTest(test.getName(), test, ()-> test.state == TestState.PASSED));
        }

        /**
         * Adds a System test to be run.
         * @param test A System test for the system.
         */
        public void addTest(SystemsTest test) {
            systemsTests.add(test);
        }

        @Override
        public void initialize() {
            curIndex = 0;
            state = TestState.RUNNING;
            testToSchedule = null;
            passTimer.stop();
            passTimer.reset();
            Log.info(name, "Test Running");

            for(SystemsTest test : systemsTests) {
                test.testState = TestState.FAILED;
            }

            if (systemsTests.size() == 0) state = TestState.FAILED;
            else {
                final SystemsTest initialTest = systemsTests.get(0);
                initialTest.testState = TestState.RUNNING;
                CommandScheduler.getInstance().schedule(initialTest);
            }
        }

        // Watches the current check; when it passes, waits timeBetweenTests, then starts the next
        @Override
        public void execute() {
            if (systemsTests.size() == 0) return;
            if (passTimer.hasElapsed(timeBetweenTests) && testToSchedule != null) {
                passTimer.stop();
                passTimer.reset();
                state = TestState.RUNNING;
                CommandScheduler.getInstance().schedule(testToSchedule);
                testToSchedule = null;
            }

            final SystemsTest test = systemsTests.get(curIndex);
            switch(test.testState) {
                case FAILED:
                    state = TestState.FAILED;
                    return;
                case PASSED:
                    curIndex ++;
                    state = TestState.PASSED;
                    if (curIndex < systemsTests.size()) {
                        passTimer.restart();
                        testToSchedule = systemsTests.get(curIndex);
                        testToSchedule.testState = TestState.RUNNING;
                    }
                    return;
                default:
            }
        }

        @Override
        public void end(boolean interrupted) {
            Log.info(name, "Test Ended");
            Log.info(name, "Test " + state);
        }

        @Override
        public boolean isFinished() {
            return state == TestState.FAILED || curIndex == systemsTests.size();
        }

        /**
         * Returns whether or not the test has passed, failed, or is running.
         * @return The current state of the test.
         */
        public TestState getTestState() {
            return state;
        }

        /**
         * Sets the time between each test.
         * @param time Time between tests.
         */
        public void setTimeBetweenTests(double time) {
            timeBetweenTests = time;
        }
    }

    /** Where a test is up to. A test that hasn't run yet counts as FAILED. */
    public enum TestState {
        FAILED,
        RUNNING,
        PASSED
    }

    private static Tester instance;

    /** The one Tester for the robot. */
    public static synchronized Tester getInstance() {
        if (instance == null) {
            instance = new Tester();
        }
        return instance;
    }

    private final HashMap<String, Test> systemTests;

    private Tester() {
        systemTests = new HashMap<String, Test>();
    }

    /**
     * Adds a Systems test to be run for a system.
     * @param name Name of the test or system.
     * @param test Systems test to be added.
     */
    public void addTest(String name, SystemsTest test) {
        if (!systemTests.containsKey(name)) {
            systemTests.put(name, new Test(name));
        }
        systemTests.get(name).addTest(test);
    }

    /**
     * Adds a Systems test to be run for a system.
     * @param name Name of the test or system.
     * @param test Systems test to be added.
     */
    public void addTest(String name, Test test) {
        if (!systemTests.containsKey(name)) {
            systemTests.put(name, new Test(name));
        }
        systemTests.get(name).addTest(test);
    }

    /**
     * Returns a Test for the robot.
     * @param name Name of the test or system.
     * @return A Test.
     */
    public Test getTest(String name) {
        return systemTests.get(name);
    
    }

    /**
     * Runs the Systems tests for a system.
     * @param name Name of the test or system.
     */
    public void runTest(String name) {
        CommandScheduler.getInstance().schedule(systemTests.get(name));
    }
}
