package common.utility.tester;

import java.util.function.BooleanSupplier;
import java.util.HashMap;
import java.util.ArrayList;

import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Team 3128's Tester utility class used to run system checks at competitions.
 */
public class Tester {
    
    /**System test */
    public static class UnitTest extends Command {

        protected String testName;
        protected Command command;
        protected BooleanSupplier passCondition;
        protected TestState testState;

        /**
         * Creates a new Unit Test.
         * @param testName Name of the test.
         * @param command Command to be run for the test.
         */
        public UnitTest(String testName, Command command) {
            this(testName, command, ()-> true);
        }

        /**
         * Creates a new Unit Test.
         * @param testName Name of the test.
         * @param command Command to be run for the test.
         * @param passCondition Condition for the test to pass.
         */
        public UnitTest(String testName, Command command, BooleanSupplier passCondition) {
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

    /**Collection of tests to be run for a system */
    public static class Test extends Command {
        private final ArrayList<UnitTest> unitTests;
        private final String name;
        private final Timer passTimer;
        private double timeBetweenTests;
        private UnitTest testToSchedule;
        private TestState state;
        private int curIndex;

        /**
         * Creates a test for a specific system.
         * @param name Name of the test or system.
         */
        private Test(String name) {
            unitTests = new ArrayList<UnitTest>();
            this.name = name;
            this.timeBetweenTests = 0;
            state = TestState.FAILED;
            curIndex = 0;
            passTimer = new Timer();
            NarwhalDashboard.getInstance().addUpdate(name, ()-> state.toString());
            NarwhalDashboard.getInstance().addButton(name, (boolean pressed) -> {
                if (pressed) this.schedule();
            });
        }

        /**
         * Adds a larger system test to be run.
         * @param test Test for the robot.
         */
        public void addTest(Test test) {
            unitTests.add(new UnitTest(test.getName(), test, ()-> test.state == TestState.PASSED));
        }

        /**
         * Adds a unit test to be run.
         * @param test A unit test for the system.
         */
        public void addTest(UnitTest test) {
            unitTests.add(test);
        }

        @Override
        public void initialize() {
            curIndex = 0;
            state = TestState.RUNNING;
            testToSchedule = null;
            passTimer.stop();
            passTimer.reset();
            Log.info(name, "Test Running");

            for(UnitTest test : unitTests) {
                test.testState = TestState.FAILED;
            }

            if (unitTests.size() == 0) state = TestState.FAILED;
            else {
                final UnitTest initialTest = unitTests.get(0);
                initialTest.testState = TestState.RUNNING;
                initialTest.schedule();
            }
        }

        @Override
        public void execute() {
            if (unitTests.size() == 0) return;
            if (passTimer.hasElapsed(timeBetweenTests) && testToSchedule != null) {
                passTimer.stop();
                passTimer.reset();
                state = TestState.RUNNING;
                testToSchedule.testState = TestState.RUNNING;
                testToSchedule.schedule();
                testToSchedule = null;
            }

            final UnitTest test = unitTests.get(curIndex);
            switch(test.testState) {
                case FAILED:
                    state = TestState.FAILED;
                    return;
                case PASSED:
                    curIndex ++;
                    state = TestState.PASSED;
                    if (curIndex < unitTests.size()) {
                        passTimer.restart();
                        testToSchedule = unitTests.get(curIndex);
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
            return state == TestState.FAILED || curIndex == unitTests.size();
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

    /**Enum representing test states */
    public enum TestState {
        FAILED,
        RUNNING,
        PASSED
    }

    private static Tester instance;

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
     * Adds a unit test to be run for a system.
     * @param name Name of the test or system.
     * @param test Unit test to be added.
     */
    public void addTest(String name, UnitTest test) {
        if (!systemTests.containsKey(name)) {
            systemTests.put(name, new Test(name));
        }
        systemTests.get(name).addTest(test);
    }

    /**
     * Adds a unit test to be run for a system.
     * @param name Name of the test or system.
     * @param test Unit test to be added.
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
     * Runs the unit tests for a system.
     * @param name Name of the test or system.
     */
    public void runTest(String name) {
        systemTests.get(name).schedule();
    }
}
