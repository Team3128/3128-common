package common.hardware.motorcontroller;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfigAccessor;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_CANSpark.ControllerType;
import common.hardware.motorcontroller.NAR_CANSpark.EncoderType;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.hardware.motorcontroller.NAR_Motor.StatusFrames;
import common.hardware.motorcontroller.sim.RollerSimModel;
import common.utility.narwhaldashboard.NarwhalDashboard;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

/** Checks {@link NAR_CANSpark} against REV's simulated Spark MAX and Spark Flex. */
class NAR_CANSparkTest {

    private static final double TOLERANCE = 1e-6;
    // One unit is two rotations of the motor; speeds per second
    private static final MotorConfig CONFIG = new MotorConfig(0.5, 60, 40, false, Neutral.BRAKE);

    private static int nextId = 10;
    private final List<NAR_CANSpark> created = new ArrayList<>();

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @AfterEach
    void closeSparks() {
        created.forEach(NAR_CANSpark::close);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    private NAR_CANSpark spark() {
        return track(new NAR_CANSpark(nextId++));
    }

    private NAR_CANSpark track(NAR_CANSpark spark) {
        created.add(spark);
        return spark;
    }

    /** What the Spark itself has. */
    private static SparkBaseConfigAccessor readBack(NAR_CANSpark spark) {
        return spark.getMotor() instanceof SparkFlex flex ? flex.configAccessor : ((SparkMax) spark.getMotor()).configAccessor;
    }

    @Test
    void configMotorSendsEveryHardwareSetting() {
        final NAR_CANSpark spark = spark();
        spark.configMotor(new MotorConfig(1, 60, 33, 44, 11, true, Neutral.COAST, StatusFrames.DEFAULT));

        final SparkBaseConfigAccessor config = readBack(spark);
        assertEquals(33, config.getSmartCurrentLimit());
        assertEquals(44, config.getSecondaryCurrentLimit(), TOLERANCE);
        assertEquals(11, config.getVoltageCompensation(), TOLERANCE);
        assertTrue(config.getInverted());
        assertEquals(IdleMode.kCoast, config.getIdleMode());
    }

    @Test
    void eachSettingIsSentAsItChanges() {
        final NAR_CANSpark spark = spark();
        spark.setStatorLimit(20);
        spark.setSupplyLimit(30);
        spark.enableVoltageCompensation(10);
        spark.setInverted(true);
        spark.setNeutralMode(Neutral.COAST);

        SparkBaseConfigAccessor config = readBack(spark);
        assertEquals(20, config.getSmartCurrentLimit());
        assertEquals(30, config.getSecondaryCurrentLimit(), TOLERANCE);
        assertEquals(10, config.getVoltageCompensation(), TOLERANCE);
        assertTrue(config.getInverted());
        assertEquals(IdleMode.kCoast, config.getIdleMode());

        spark.setNeutralMode(Neutral.BRAKE);
        assertEquals(IdleMode.kBrake, readBack(spark).getIdleMode());
    }

    @Test
    void smallerMotorsGetLowerStartingLimits() {
        final NAR_CANSpark spark = track(new NAR_CANSpark(nextId++, ControllerType.CAN_SPARK_MAX, MotorType.kBrushed));
        // The starting limits are stored, and sent with the first configuration
        spark.configure();
        assertEquals(MotorControllerConstants.NEO_STATOR_550CurrentLimit, readBack(spark).getSmartCurrentLimit());
    }

    @Test
    void positionGainsAreConvertedToTheSparksUnits() {
        final NAR_CANSpark spark = spark();
        spark.configMotor(CONFIG);

        spark.configClosedLoop(new PIDFFConfig(2, 3, 4, 0.1, 0.2, 0.3), Control.Position);

        final SparkBaseConfigAccessor config = readBack(spark);
        // Volts per unit to output fraction per rotation, and the 1 ms loop for kI and kD
        assertEquals(2 * 0.5 / 12, config.closedLoop.getP(), 1e-6);
        assertEquals(3 * 0.5 / 12 * 0.001, config.closedLoop.getI(), 1e-9);
        assertEquals(4 * 0.5 / 12 / 0.001, config.closedLoop.getD(), 1e-3);
        assertEquals(0.1, config.closedLoop.feedForward.getkS(), 1e-6);
        // Volts per unit per second to volts per RPM
        assertEquals(0.2 * 0.5 / 60, config.closedLoop.feedForward.getkV(), 1e-6);
        // kG isn't checked: REV's simulated Spark reads it back as 0, though kCos works
        assertEquals(0, config.closedLoop.feedForward.getkCos(), 1e-6);
    }

    @Test
    void armGainsUseTheCosineInArmRotations() {
        final NAR_CANSpark spark = spark();
        spark.configMotor(CONFIG);

        spark.configArmClosedLoop(new PIDFFConfig(1, 0, 0, 0, 0, 0.4), 50);

        final SparkBaseConfigAccessor config = readBack(spark);
        assertEquals(0.4, config.closedLoop.feedForward.getkCos(), 1e-6);
        assertEquals(1.0 / 50, config.closedLoop.feedForward.getkCosRatio(), 1e-6);
        assertEquals(0, config.closedLoop.feedForward.getkG(), 1e-6);
    }

    @Test
    void pidGainsCanBeSentAsTheyAre() {
        final NAR_CANSpark spark = track(new NAR_CANSpark(nextId++, ControllerType.CAN_SPARK_MAX, MotorType.kBrushless,
            EncoderType.Relative, new PIDFFConfig(3, 4, 5)));
        spark.configure();
        assertEquals(3, readBack(spark).closedLoop.getP(), 1e-6);

        spark.configPID(new PIDFFConfig(6, 0, 0));
        assertEquals(6, readBack(spark).closedLoop.getP(), 1e-6);
    }

    @Test
    void everyKindOfRequestIsAccepted() {
        // The unit conversions are checked in NAR_MotorTest; the simulated Spark only reports what
        // it's doing once simulation steps it, so this checks each request goes through
        final NAR_CANSpark spark = spark();
        spark.configMotor(CONFIG);

        spark.set(0.5);
        assertEquals(REVLibError.kOk, spark.getMotor().getLastError());
        spark.set(30, Control.Velocity, 1.5);
        assertEquals(REVLibError.kOk, spark.getMotor().getLastError());
        spark.set(2.5, Control.Position);
        assertEquals(REVLibError.kOk, spark.getMotor().getLastError());
    }

    @Test
    void followsAnotherSparkOnTheSpark() {
        final NAR_CANSpark leader = spark();
        final NAR_CANSpark follower = spark();

        assertTrue(follower.followOnController(leader, true));

        assertEquals(leader.getMotor().getDeviceId(), readBack(follower).getFollowerModeLeaderId());
        assertTrue(readBack(follower).getFollowerModeInverted());
    }

    @Test
    void followingLeadersOfEitherBrand() {
        final NAR_CANSpark leader = spark();
        final NAR_CANSpark follower = spark();

        follower.follow(leader);
        assertEquals(leader.getMotor().getDeviceId(), readBack(follower).getFollowerModeLeaderId());
        assertFalse(readBack(follower).getFollowerModeInverted());

        // A different brand can only be followed from robot code
        final NAR_CANSpark other = spark();
        assertFalse(other.followOnController(MockMotor.talonLike(1), false));
        other.follow(MockMotor.talonLike(2));
    }

    // The Spark groups signals into frames, and a frame is sent as often as its most frequent
    // signal asks, so these check each signal is reported at least as often as it was set.

    @Test
    void statusFramePresetsKeepWhatsReadReporting() {
        final NAR_CANSpark spark = spark();

        spark.setPositionStatusFrames();
        SparkBaseConfigAccessor config = readBack(spark);
        assertTrue(config.signals.getPrimaryEncoderPositionPeriodMs() <= MotorControllerConstants.HIGH_PRIORITY);
        assertTrue(config.signals.getAppliedOutputPeriodMs() <= MotorControllerConstants.HIGH_PRIORITY);
        // Still reports current and temperature for the logs
        assertTrue(config.signals.getOutputCurrentPeriodMs() <= MotorControllerConstants.MEDIUM_PRIORITY);
        assertTrue(config.signals.getMotorTemperaturePeriodMs() <= MotorControllerConstants.MEDIUM_PRIORITY);

        spark.setVelocityStatusFrames();
        config = readBack(spark);
        assertTrue(config.signals.getPrimaryEncoderVelocityPeriodMs() <= MotorControllerConstants.HIGH_PRIORITY);
        assertTrue(config.signals.getPrimaryEncoderPositionPeriodMs() <= MotorControllerConstants.LOW_PRIORITY);

        spark.setDefaultStatusFrames();
        spark.setFollowerStatusFrames();
        assertTrue(readBack(spark).signals.getOutputCurrentPeriodMs() <= MotorControllerConstants.MEDIUM_PRIORITY);
    }

    @Test
    void eachFrameSetsItsOwnSignals() {
        final NAR_CANSpark spark = spark();

        spark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);
        spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 60);
        spark.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 70);
        spark.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 80); // an alternate encoder; ignored

        final SparkBaseConfigAccessor config = readBack(spark);
        assertTrue(config.signals.getAnalogPositionPeriodMs() <= 50);
        assertTrue(config.signals.getAnalogVoltagePeriodMs() <= 50);
        assertTrue(config.signals.getAbsoluteEncoderPositionPeriodMs() <= 60);
        assertTrue(config.signals.getAbsoluteEncoderVelocityPeriodMs() <= 70);
    }

    @Test
    void sparkMaxSimulationDrivesTheModel() {
        final NAR_CANSpark spark = spark();
        spark.configMotor(new MotorConfig(1, 60, 80, false, Neutral.BRAKE));
        final RollerSimModel model = new RollerSimModel(DCMotor.getNEO(1), 1, 0.001);
        spark.enableSim(model);

        runAtVolts(spark, 6);

        assertTrue(model.getRotorRPS() > 1, "the roller should spin up");
        assertTrue(spark.getVelocity() > 1, "and the Spark should report it");
        assertTrue(spark.getPosition() > 0);
    }

    @Test
    void sparkFlexSimulationDrivesTheModel() {
        final NAR_CANSpark spark = track(new NAR_CANSpark(nextId++, ControllerType.CAN_SPARK_FLEX));
        spark.configMotor(new MotorConfig(1, 60, 80, false, Neutral.BRAKE));
        final RollerSimModel model = new RollerSimModel(DCMotor.getNeoVortex(1), 1, 0.001);
        spark.enableSim(model);

        runAtVolts(spark, 6);

        assertTrue(model.getRotorRPS() > 1);
        assertTrue(Double.isFinite(spark.getTorque()));
    }

    @Test
    void absoluteEncoderIsReadInSimulationAndRunsTheControlLoop() {
        final NAR_CANSpark spark = track(new NAR_CANSpark(nextId++, ControllerType.CAN_SPARK_MAX, MotorType.kBrushless,
            EncoderType.Absolute));
        spark.configMotor(new MotorConfig(1, 60, 80, false, Neutral.BRAKE));
        assertEquals(FeedbackSensor.kAbsoluteEncoder, readBack(spark).closedLoop.getFeedbackSensor());

        // The encoder is on the arm's shaft, after a 10:1 gearbox
        spark.setMotorRotationsPerSensorRotation(10);
        spark.enableSim(new RollerSimModel(DCMotor.getNEO(1), 10, 0.01));
        NAR_Motor.updateSimulation(0.02);
        spark.resetPosition(0.25);
        NAR_Motor.updateSimulation(0.02);

        assertEquals(0.25, spark.getPosition(), 1e-3);
        assertTrue(Double.isFinite(spark.getVelocity()));
    }

    @Test
    void healthAndSensorsAreAvailable() {
        final NAR_CANSpark spark = spark();
        assertEquals(NarwhalDashboard.State.RUNNING, spark.getState());
        assertNotNull(spark.getAllFaults());
        assertTrue(Double.isFinite(spark.getAppliedOutput()));
        assertTrue(Double.isFinite(spark.getStallCurrent()));
        assertTrue(Double.isFinite(spark.getTorque()));
        assertTrue(Double.isFinite(spark.getTemperature()));
    }

    @Test
    void savingToFlashDoesNothingInSimulation() {
        spark();
        assertEquals(REVLibError.kOk, created.get(0).burnFlash());
        NAR_CANSpark.burnFlashAll();
        assertEquals(0, NAR_CANSpark.getNumFailedConfigs());
    }

    /** Drives the Spark at a voltage for half a second of simulated time. */
    private static void runAtVolts(NAR_CANSpark spark, double volts) {
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        spark.setVolts(volts);
        for (int i = 0; i < 25; i++) {
            NAR_Motor.updateSimulation(0.02);
        }
    }
}
