package common.hardware.motorcontroller;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.hardware.motorcontroller.NAR_Motor.StatusFrames;
import common.hardware.motorcontroller.sim.RollerSimModel;
import common.utility.narwhaldashboard.NarwhalDashboard;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

/**
 * Checks {@link NAR_TalonFX} against CTRE's simulated TalonFX: what it sends, and what it reads.
 * The simulated Talon runs on its own thread, so tests wait briefly for it to catch up.
 */
class NAR_TalonFXTest {

    private static final double TOLERANCE = 1e-6;
    // The Talon stores gains to about a thousandth
    private static final double GAIN_TOLERANCE = 1e-3;
    // One unit is two rotations of the motor; speeds per second
    private static final MotorConfig CONFIG = new MotorConfig(0.5, 60, 40, false, Neutral.BRAKE);

    private static int nextId = 10;
    private final List<NAR_TalonFX> created = new ArrayList<>();

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @AfterEach
    void closeTalons() {
        created.forEach(NAR_TalonFX::close);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    private NAR_TalonFX talon() {
        final NAR_TalonFX talon = new NAR_TalonFX(nextId++);
        created.add(talon);
        return talon;
    }

    /** What the Talon itself has, read back from it. */
    private static TalonFXConfiguration readBack(NAR_TalonFX talon) {
        final TalonFXConfiguration config = new TalonFXConfiguration();
        talon.getMotor().getConfigurator().refresh(config);
        return config;
    }

    /** Waits for the simulated Talon to report new values. */
    private static void waitForTalon() {
        Timer.delay(0.15);
    }

    @Test
    void configMotorSendsEveryHardwareSetting() {
        final NAR_TalonFX talon = talon();
        talon.configMotor(new MotorConfig(1, 60, 33, 44, 11, true, Neutral.COAST, StatusFrames.DEFAULT));

        final TalonFXConfiguration config = readBack(talon);
        assertEquals(33, config.CurrentLimits.StatorCurrentLimit, TOLERANCE);
        assertTrue(config.CurrentLimits.StatorCurrentLimitEnable);
        assertEquals(44, config.CurrentLimits.SupplyCurrentLimit, TOLERANCE);
        assertEquals(11, config.Voltage.PeakForwardVoltage, TOLERANCE);
        assertEquals(-11, config.Voltage.PeakReverseVoltage, TOLERANCE);
        assertEquals(InvertedValue.Clockwise_Positive, config.MotorOutput.Inverted);
        assertEquals(NeutralModeValue.Coast, config.MotorOutput.NeutralMode);
    }

    @Test
    void eachSettingIsSentAsItChanges() {
        final NAR_TalonFX talon = talon();
        talon.setStatorLimit(20);
        talon.setSupplyLimit(30);
        talon.enableVoltageCompensation(10);
        talon.setInverted(true);
        talon.setNeutralMode(Neutral.COAST);

        TalonFXConfiguration config = readBack(talon);
        assertEquals(20, config.CurrentLimits.StatorCurrentLimit, TOLERANCE);
        assertEquals(30, config.CurrentLimits.SupplyCurrentLimit, TOLERANCE);
        assertEquals(10, config.Voltage.PeakForwardVoltage, TOLERANCE);
        assertEquals(InvertedValue.Clockwise_Positive, config.MotorOutput.Inverted);
        assertEquals(NeutralModeValue.Coast, config.MotorOutput.NeutralMode);

        talon.setCurrentLimit(25, 35);
        talon.setNeutralMode(Neutral.BRAKE);
        config = readBack(talon);
        assertEquals(25, config.CurrentLimits.StatorCurrentLimit, TOLERANCE);
        assertEquals(35, config.CurrentLimits.SupplyCurrentLimit, TOLERANCE);
        assertEquals(NeutralModeValue.Brake, config.MotorOutput.NeutralMode);
    }

    @Test
    void positionGainsAreConvertedToTheTalonsUnits() {
        final NAR_TalonFX talon = talon();
        talon.configMotor(CONFIG);

        talon.configClosedLoop(new PIDFFConfig(2, 0, 0, 0.1, 0.2, 0.3), Control.Position);

        final TalonFXConfiguration config = readBack(talon);
        // 2 volts per unit is 1 volt per rotation
        assertEquals(1, config.Slot0.kP, GAIN_TOLERANCE);
        assertEquals(0.1, config.Slot0.kS, GAIN_TOLERANCE);
        // 0.2 volts per unit per second is 0.1 volts per rotation per second
        assertEquals(0.1, config.Slot0.kV, GAIN_TOLERANCE);
        assertEquals(0.3, config.Slot0.kG, GAIN_TOLERANCE);
        assertEquals(GravityTypeValue.Elevator_Static, config.Slot0.GravityType);
        assertEquals(StaticFeedforwardSignValue.UseClosedLoopSign, config.Slot0.StaticFeedforwardSign);
    }

    @Test
    void velocityGainsSignKSByVelocity() {
        final NAR_TalonFX talon = talon();
        talon.configMotor(CONFIG);
        talon.configClosedLoop(new PIDFFConfig(2), Control.Velocity);
        assertEquals(StaticFeedforwardSignValue.UseVelocitySign, readBack(talon).Slot0.StaticFeedforwardSign);
    }

    @Test
    void armGainsUseTheCosineInArmRotations() {
        final NAR_TalonFX talon = talon();
        talon.configMotor(CONFIG);

        talon.configArmClosedLoop(new PIDFFConfig(1, 0, 0, 0, 0, 0.4), 50);

        final TalonFXConfiguration config = readBack(talon);
        assertEquals(GravityTypeValue.Arm_Cosine, config.Slot0.GravityType);
        assertEquals(50, config.Feedback.SensorToMechanismRatio, TOLERANCE);
        assertEquals(0.4, config.Slot0.kG, GAIN_TOLERANCE);
    }

    @Test
    void pidGainsCanBeSentAsTheyAre() {
        final NAR_TalonFX talon = new NAR_TalonFX(nextId++, "rio", new PIDFFConfig(3, 4, 5));
        created.add(talon);

        final TalonFXConfiguration config = readBack(talon);
        assertEquals(3, config.Slot0.kP, GAIN_TOLERANCE);
        assertEquals(4, config.Slot0.kI, GAIN_TOLERANCE);
        assertEquals(5, config.Slot0.kD, GAIN_TOLERANCE);

        talon.configPID(new PIDFFConfig(6, 0, 0));
        assertEquals(6, readBack(talon).Slot0.kP, TOLERANCE);
    }

    @Test
    void requestsAreSentInTheTalonsUnits() {
        final NAR_TalonFX talon = talon();
        talon.configMotor(CONFIG);

        talon.set(0.5);
        assertEquals(0.5, ((DutyCycleOut) talon.getMotor().getAppliedControl()).Output, TOLERANCE);

        // 30 units per second is 60 rotations per second
        talon.set(30, Control.Velocity, 1.5);
        final VelocityVoltage velocity = (VelocityVoltage) talon.getMotor().getAppliedControl();
        assertEquals(60, velocity.Velocity, TOLERANCE);
        assertEquals(1.5, velocity.FeedForward, TOLERANCE);

        talon.set(2.5, Control.Position, 0.5);
        final PositionVoltage position = (PositionVoltage) talon.getMotor().getAppliedControl();
        assertEquals(5, position.Position, TOLERANCE);
        assertEquals(0.5, position.FeedForward, TOLERANCE);
    }

    @Test
    void focRequestsEnableFoc() {
        final NAR_TalonFX talon = talon();
        talon.configMotor(CONFIG);

        talon.setVoltageFOC(6);
        final VoltageOut voltage = (VoltageOut) talon.getMotor().getAppliedControl();
        assertEquals(6, voltage.Output, TOLERANCE);
        assertTrue(voltage.EnableFOC);

        talon.setVelocityFOC(120, 1);
        VelocityVoltage velocity = (VelocityVoltage) talon.getMotor().getAppliedControl();
        assertEquals(2, velocity.Velocity, TOLERANCE); // 120 RPM is 2 rotations per second
        assertTrue(velocity.EnableFOC);

        talon.setVelocityFOC2(30, 1);
        velocity = (VelocityVoltage) talon.getMotor().getAppliedControl();
        assertEquals(60, velocity.Velocity, TOLERANCE); // in this motor's units, like set()

        talon.setPositionFOC(3, 0.5);
        final PositionVoltage position = (PositionVoltage) talon.getMotor().getAppliedControl();
        assertEquals(3, position.Position, TOLERANCE);
        assertTrue(position.EnableFOC);
    }

    @Test
    void followsAnotherTalonOnTheTalon() {
        final NAR_TalonFX leader = talon();
        final NAR_TalonFX follower = talon();

        assertTrue(follower.followOnController(leader, true));

        final Follower request = (Follower) follower.getMotor().getAppliedControl();
        assertEquals(leader.getMotor().getDeviceID(), request.LeaderID);
        assertEquals(MotorAlignmentValue.Opposed, request.MotorAlignment);
    }

    @Test
    void cantFollowADifferentBrandOnTheTalon() {
        assertFalse(talon().followOnController(MockMotor.sparkLike(1), false));
    }

    @Test
    void readsPositionAndVelocityInYourUnits() {
        final NAR_TalonFX talon = talon();
        talon.configMotor(CONFIG);

        talon.getMotor().getSimState().setRawRotorPosition(5);
        talon.getMotor().getSimState().setRotorVelocity(10);
        waitForTalon();

        assertEquals(2.5, talon.getPosition(), 1e-3); // 5 rotations is 2.5 units
        assertEquals(5, talon.getVelocity(), 1e-3);   // 10 rotations per second is 5 units per second

        talon.resetPosition(1);
        waitForTalon();
        assertEquals(1, talon.getPosition(), 1e-3);
    }

    @Test
    void positionMotorStillReportsItsSpeed() {
        final NAR_TalonFX talon = talon();
        talon.configMotor(new MotorConfig(1, 60, 40, false, Neutral.BRAKE, StatusFrames.POSITION));

        talon.getMotor().getSimState().setRotorVelocity(10);
        Timer.delay(0.4); // reported less often than position

        assertEquals(10, talon.getVelocity(), 1e-3);
    }

    @Test
    void everyStatusFramePresetLeavesItRunning() {
        final NAR_TalonFX talon = talon();
        talon.setDefaultStatusFrames();
        talon.setPositionStatusFrames();
        talon.setVelocityStatusFrames();
        talon.setFollowerStatusFrames();
        waitForTalon();

        assertEquals(NarwhalDashboard.State.RUNNING, talon.getState());
        assertEquals(0, NAR_TalonFX.getNumFailedConfigs());
    }

    @Test
    void sensorReadingsAreAvailable() {
        final NAR_TalonFX talon = talon();
        waitForTalon();
        assertTrue(Double.isFinite(talon.getAppliedOutput()));
        assertTrue(Double.isFinite(talon.getStallCurrent()));
        assertTrue(Double.isFinite(talon.getTorque()));
        assertTrue(Double.isFinite(talon.getTemperature()));
    }

    @Test
    void simulationDrivesTheModelWithTheTalonsOutput() {
        final NAR_TalonFX talon = talon();
        talon.configMotor(new MotorConfig(1, 60, 80, false, Neutral.BRAKE));
        final RollerSimModel model = new RollerSimModel(DCMotor.getKrakenX60(1), 1, 0.001);
        talon.enableSim(model);

        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        talon.setVolts(6);
        for (int i = 0; i < 25; i++) {
            Timer.delay(0.02);
            NAR_Motor.updateSimulation(0.02);
        }

        assertTrue(model.getRotorRPS() > 1, "the roller should spin up");
        assertInstanceOf(DutyCycleOut.class, talon.getMotor().getAppliedControl());
    }
}
