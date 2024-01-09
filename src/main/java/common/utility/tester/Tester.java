package common.utility.tester;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import java.util.function.BooleanSupplier;
import java.util.HashMap;
import java.util.ArrayList;

import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Team 3128's Tester utility class used to run system checks at competitions.
 */
public class Tester {
    
    /**System test */
    public static class UnitTest extends WaitCommand {

        protected String testName;
        protected Command command;
        protected BooleanSupplier passCondition;
        protected TestState testState;
        protected double plateau;
        protected double timeOut;
        private Timer passTimer;

        /**
         * Creates a unit test.
         * @param testName Name of the test.
         * @param command Command to run for the test.
         * @param passCondition Condition to see if the test passed.
         * @param plateau How long the pass condition needs to be true.
         * @param timeOut Time the test has to run before failing.
         * @param requirements Subsystems involved in the test.
         */
        public UnitTest(String testName, Command command, BooleanSupplier passCondition, double plateau, double timeOut, Subsystem... requirements) {
            super(timeOut);
            this.testName = testName;
            this.command = command;
            this.passCondition = passCondition;
            this.timeOut = timeOut;
            testState = TestState.FAILED;
            passTimer = new Timer();
            addRequirements(requirements);
        }

        @Override
        public void initialize() {
            Log.info(testName, "Test Running");
            super.initialize();
            command.initialize();
            passTimer.reset();
            testState = TestState.RUNNING;
        }

        @Override
        public void execute() {
            command.execute();
            if (!passCondition.getAsBoolean()) {
                passTimer.reset();
            }
        }

        public void end(boolean interrupted) {
            testState = TestState.FAILED;
            if (passCondition.getAsBoolean()) {
                testState = TestState.PASSED;
            }
            Log.info(testName, "Test " + testState.toString());
        }

        @Override
        public boolean isFinished() {
            return super.isFinished() || passTimer.hasElapsed(plateau);
        }
    }

    /**Collection of tests to be run for a system */
    public class Test extends Command {
        private final ArrayList<UnitTest> unitTests;
        private final String name;
        private TestState state;
        private int curIndex;

        /**
         * Creates a test for a specific system.
         * @param name Name of the test or system.
         */
        private Test(String name) {
            unitTests = new ArrayList<UnitTest>();
            this.name = name;
            state = TestState.FAILED;
            curIndex = 0;
            NarwhalDashboard.getInstance().addUpdate(name, ()-> state);
            NarwhalDashboard.getInstance().addButton(name, (boolean pressed) -> {
                if (pressed) this.schedule();
            });
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
            Log.info(name, "TEST RUNNING");
            if (unitTests.size() == 0) state = TestState.FAILED;
            else unitTests.get(0).schedule();
        }

        @Override
        public void execute() {
            if (unitTests.size() == 0) return;

            final UnitTest test = unitTests.get(curIndex);
            switch(test.testState) {
                case FAILED:
                    state = TestState.FAILED;
                    return;
                case PASSED:
                    curIndex ++;
                    if (curIndex == unitTests.size()) {
                        state = TestState.PASSED;
                        return;
                    }
                    unitTests.get(curIndex).beforeStarting(waitSeconds(2)).schedule();
                    break;
                default:
            }
        }

        @Override
        public void end(boolean interrupted) {
            Log.info(name, "TEST " + state);
        }

        @Override
        public boolean isFinished() {
            return state != TestState.RUNNING;
        }

        public TestState getTestState() {
            return state;
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

    private Tester() {}

    public HashMap<String, Test> systemTests = new HashMap<String, Test>();

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
     * Runs the unit tests for a system.
     * @param name Name of the test or system.
     */
    public void runTest(String name) {
        systemTests.get(name).schedule();
    }
}
