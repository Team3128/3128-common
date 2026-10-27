package common.core.fsm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import common.core.controllers.PIDFFConfig;
import common.core.subsystems.LinearMechanism;
import common.hardware.motorcontroller.FakeMotor;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

class FSMSubsystemBaseTest {

    enum TestState { IDLE, INTAKE, SHOOT }

    /** A state machine whose transitions are whatever the test registers. */
    static class TestFSM extends FSMSubsystemBase<TestState> {
        final TransitionMap<TestState> map;
        final Consumer<TransitionMap<TestState>> register;
        int registrations = 0;

        TestFSM(TransitionMap<TestState> map, Consumer<TransitionMap<TestState>> register) {
            super(TestState.class, map, TestState.IDLE);
            this.map = map;
            this.register = register;
        }

        @Override
        public void registerTransitions() {
            registrations++;
            register.accept(map);
        }
    }

    private static TestFSM fsm(Consumer<TransitionMap<TestState>> register) {
        return new TestFSM(new TransitionMap<>(TestState.class), register);
    }

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void enableRobot() {
        // The scheduler only runs commands while the robot is enabled
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    @AfterEach
    void resetScheduler() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    @Test
    void validTransitionRunsItsCommandAndChangesState() {
        final AtomicBoolean ran = new AtomicBoolean();
        final TestFSM fsm = fsm(map -> map.addTransition(TestState.IDLE, TestState.INTAKE, () -> ran.set(true)));

        fsm.setState(TestState.INTAKE);

        assertTrue(ran.get());
        assertEquals(TestState.INTAKE, fsm.getState());
        assertEquals(TestState.IDLE, fsm.getPreviousState());
        assertTrue(fsm.stateEquals(TestState.INTAKE));
    }

    @Test
    void invalidTransitionLeavesTheStateAlone() {
        final TestFSM fsm = fsm(map -> map.addTransition(TestState.IDLE, TestState.INTAKE, Commands.none()));

        fsm.setState(TestState.SHOOT);

        assertEquals(TestState.IDLE, fsm.getState());
        assertNull(fsm.getRequestTransition());
    }

    @Test
    void nullStateIsIgnored() {
        final TestFSM fsm = fsm(map -> map.addTransition(TestState.IDLE, TestState.INTAKE, Commands.none()));

        fsm.setState(null);

        assertEquals(TestState.IDLE, fsm.getState());
    }

    @Test
    void transitionsAreRegisteredOnceOnFirstUse() {
        final TestFSM fsm = fsm(map -> map.addCommutativeTransition(TestState.IDLE, TestState.INTAKE));
        assertEquals(0, fsm.registrations);

        fsm.setState(TestState.INTAKE);
        fsm.setState(TestState.IDLE);

        assertEquals(1, fsm.registrations);
    }

    @Test
    void newTransitionInterruptsOneStillRunning() {
        final AtomicBoolean interrupted = new AtomicBoolean();
        final Command slowTransition = Commands.idle().finallyDo(wasInterrupted -> interrupted.set(wasInterrupted));
        final TestFSM fsm = fsm(map -> {
            map.addTransition(TestState.IDLE, TestState.INTAKE, slowTransition);
            map.addTransition(TestState.INTAKE, TestState.SHOOT, Commands.none());
        });

        fsm.setState(TestState.INTAKE);
        assertTrue(fsm.isTransitioning());

        fsm.setState(TestState.SHOOT);

        assertFalse(slowTransition.isScheduled());
        assertTrue(interrupted.get());
        assertEquals(TestState.SHOOT, fsm.getState());
    }

    @Test
    void finishedTransitionIsNotTransitioning() {
        final TestFSM fsm = fsm(map -> map.addTransition(TestState.IDLE, TestState.INTAKE, Commands.none()));

        fsm.setState(TestState.INTAKE);
        CommandScheduler.getInstance().run();

        assertFalse(fsm.isTransitioning());
    }

    @Test
    void stopInterruptsTheTransitionAndStopsMechanisms() {
        final Command slowTransition = Commands.idle();
        final TestFSM fsm = fsm(map -> map.addTransition(TestState.IDLE, TestState.INTAKE, slowTransition));
        final FakeMotor motor = FakeMotor.talonLike(1);
        final LinearMechanism slider = new LinearMechanism("FSMSubsystemBaseTestSlider",
            new MotorConfig(0.01, 60, 40, false, Neutral.BRAKE), new PIDFFConfig(20), 0.005, 0, 0.3, 1, 0, 2,
            DCMotor.getKrakenX60(1), motor);
        fsm.addMechanisms(slider);
        fsm.setState(TestState.INTAKE);
        slider.setSetpoint(0.2);
        slider.periodic(); // the motor is now holding a position

        fsm.stop();

        assertFalse(slowTransition.isScheduled());
        assertFalse(slider.isEnabled());
        assertEquals(0, motor.lastPercentOutput, 1e-9);
    }

    @Test
    void mechanismsCanBeFoundByName() {
        final TestFSM fsm = fsm(map -> {});
        final LinearMechanism slider = new LinearMechanism("FSMSubsystemBaseTestNamed",
            new MotorConfig(0.01, 60, 40, false, Neutral.BRAKE), new PIDFFConfig(20), 0.005, 0, 0.3, 1, 0, 2,
            DCMotor.getKrakenX60(1), FakeMotor.talonLike(1));
        fsm.addMechanisms(slider);

        assertSame(slider, fsm.getMechanism("FSMSubsystemBaseTestNamed"));
        assertNull(fsm.getMechanism("Missing"));
    }

    @Test
    void overrideStateSkipsTheTransition() {
        final AtomicBoolean ran = new AtomicBoolean();
        final TestFSM fsm = fsm(map -> map.addTransition(TestState.IDLE, TestState.SHOOT, () -> ran.set(true)));

        fsm.overrideState(TestState.SHOOT);

        assertFalse(ran.get());
        assertEquals(TestState.SHOOT, fsm.getState());
        assertEquals(TestState.IDLE, fsm.getPreviousState());
    }
}
