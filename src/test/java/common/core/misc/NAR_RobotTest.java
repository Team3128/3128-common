package common.core.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;

/** Runs {@link NAR_Robot}'s main loop for a moment, then stops it. */
class NAR_RobotTest {

    /** A robot that counts what's called. */
    private static class TestRobot extends NAR_Robot {
        final AtomicInteger robotInits = new AtomicInteger();
        final AtomicInteger simulationInits = new AtomicInteger();
        final AtomicInteger loops = new AtomicInteger();

        @Override public void robotInit() { robotInits.incrementAndGet(); }
        @Override public void simulationInit() { simulationInits.incrementAndGet(); }
        @Override public void robotPeriodic() { loops.incrementAndGet(); }
    }

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @Test
    void runsTheRobotLoopAndAddedCallbacks() throws InterruptedException {
        final TestRobot robot = new TestRobot();
        final AtomicInteger callbacks = new AtomicInteger();
        NAR_Robot.addPeriodic(callbacks::incrementAndGet, 0.01, 0.005);

        final Thread main = new Thread(robot::startCompetition);
        main.start();
        for (int i = 0; i < 100 && robot.loops.get() < 5; i++) Thread.sleep(20);
        robot.endCompetition();
        main.join(2000);

        assertFalse(main.isAlive(), "endCompetition should stop the loop");
        assertEquals(1, robot.robotInits.get());
        assertEquals(1, robot.simulationInits.get());
        assertTrue(robot.loops.get() >= 5);
        assertTrue(callbacks.get() >= 5);
        robot.close();
    }

    @Test
    void callbacksAreOrderedByWhenTheyNextRun() {
        // When a callback next runs depends on the time it's made, so stop the clock while making them
        SimHooks.pauseTiming();
        final NAR_Robot.Callback soon = new NAR_Robot.Callback(() -> {}, 0, 0.02, 0);
        final NAR_Robot.Callback later = new NAR_Robot.Callback(() -> {}, 0, 0.02, 0.01);
        final NAR_Robot.Callback sameAsSoon = new NAR_Robot.Callback(() -> {}, 0, 0.02, 0);
        SimHooks.resumeTiming();

        assertTrue(soon.compareTo(later) < 0);
        assertEquals(soon, sameAsSoon);
        assertEquals(soon.hashCode(), sameAsSoon.hashCode());
        assertNotEquals(soon, later);
        assertNotEquals(soon, "not a callback");
    }
}
