package common.core.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;

import common.core.controllers.PIDFFConfig;
import common.core.swerve.SwerveModuleConfig.SwerveEncoderConfig;
import common.core.swerve.SwerveModuleConfig.SwerveMotorConfig;
import common.hardware.motorcontroller.MockMotor;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.utility.narwhaldashboard.NarwhalDashboard;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

/**
 * Checks the team's older swerve code, {@link SwerveModule} and {@link SwerveBase}, with
 * simulated CTRE drive motors and CANcoders and mock steering motors.
 */
class SwerveTest {

    private static final double TOLERANCE = 1e-6;
    private static final double MAX_SPEED = 4; // meters per second
    // Steering in degrees on a 12.8:1 gearbox; driving in meters, 0.05 per motor rotation
    private static final MotorConfig ANGLE_CONFIG = new MotorConfig(360 / 12.8, 60, 30, false, Neutral.BRAKE);
    private static final MotorConfig DRIVE_CONFIG = new MotorConfig(0.05, 60, 60, false, Neutral.BRAKE);

    // CAN IDs only go up to 62; each test closes its devices, so each starts again here
    private int nextId = 20;
    private final List<AutoCloseable> created = new ArrayList<>();

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @AfterEach
    void closeDevices() throws Exception {
        for (AutoCloseable device : created) device.close();
    }

    /** A module whose CANcoder reads {@code cancoderDegrees}, with an offset of {@code offsetDegrees}. */
    private SwerveModuleConfig moduleConfig(int number, double cancoderDegrees, double offsetDegrees) {
        final CANcoder cancoder = new CANcoder(nextId++);
        final NAR_TalonFX drive = new NAR_TalonFX(nextId++);
        created.add(cancoder);
        created.add(drive);
        cancoder.getSimState().setRawPosition(cancoderDegrees / 360);
        Timer.delay(0.15); // for the simulated CANcoder to report it
        return new SwerveModuleConfig(number,
            new SwerveMotorConfig(drive, DRIVE_CONFIG, new PIDFFConfig(0.1, 0, 0, 0.2, 2.5, 0, 0)),
            new SwerveMotorConfig(MockMotor.talonLike(nextId++), ANGLE_CONFIG, new PIDFFConfig(0.05)),
            new SwerveEncoderConfig(cancoder, offsetDegrees, false),
            MAX_SPEED);
    }

    private static MockMotor angleMotor(SwerveModule module) {
        return (MockMotor) module.getAngleMotor();
    }

    @Test
    void moduleStartsPointingWhereItsCANcoderSays() {
        // The CANcoder reads 90 degrees, and reads 30 with the wheel straight ahead
        final SwerveModule module = new SwerveModule(moduleConfig(0, 90, 30));

        assertEquals(60, module.getAbsoluteAngle().getDegrees(), 0.1);
        assertEquals(90, module.getRawAbsoluteAngle().getDegrees(), 0.1);
        assertEquals(60, module.getState().angle.getDegrees(), 0.1);
        assertNotNull(module.getAngleEncoder());
    }

    @Test
    void moduleSteersAndDrives() {
        final SwerveModule module = new SwerveModule(moduleConfig(0, 0, 0));

        module.setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(45)));

        // 45 degrees of steering, in motor rotations
        assertEquals(45 / (360 / 12.8), angleMotor(module).lastPositionRotations, 1e-6);
        final VelocityVoltage drive = (VelocityVoltage) ((NAR_TalonFX) module.getDriveMotor()).getMotor().getAppliedControl();
        assertEquals(2 / 0.05, drive.Velocity, 1e-6); // 2 m/s is 40 rotations per second
        assertTrue(drive.EnableFOC);
        assertEquals(0.2 + 2.5 * 2, drive.FeedForward, 1e-6); // kS plus kV times the speed
    }

    @Test
    void moduleDoesntSteerWhenBarelyMoving() {
        final SwerveModule module = new SwerveModule(moduleConfig(0, 0, 0));
        module.setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(30)));
        final double steered = angleMotor(module).lastPositionRotations;

        module.setDesiredState(new SwerveModuleState(0.01, Rotation2d.fromDegrees(80)));

        assertEquals(steered, angleMotor(module).lastPositionRotations, TOLERANCE);
    }

    @Test
    void moduleLocksStopsAndBrakes() {
        final SwerveModule module = new SwerveModule(moduleConfig(0, 0, 0));
        module.xLock(Rotation2d.fromDegrees(45));
        module.stop();
        assertEquals(Control.PercentOutput, angleMotor(module).lastMode);

        module.setBrakeMode(false);
        assertEquals(Neutral.COAST, angleMotor(module).neutralMode);
        module.setBrakeMode(true);
        assertEquals(Neutral.BRAKE, angleMotor(module).neutralMode);

        assertEquals(NarwhalDashboard.State.RUNNING, module.getRunningState());
        assertEquals(0, module.getModulePosition().distanceMeters, 1e-3);
    }

    /** A drivetrain whose gyro is a number the test sets. */
    private static class TestSwerve extends SwerveBase {
        double yaw = 0;

        TestSwerve(SwerveModuleConfig... configs) {
            super(new SwerveDriveKinematics(new Translation2d(0.3, 0.3), new Translation2d(0.3, -0.3),
                    new Translation2d(-0.3, 0.3), new Translation2d(-0.3, -0.3)),
                VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.9, 0.9, 0.9), configs);
        }

        @Override public double getYaw() { return yaw; }
        @Override public double getPitch() { return 1; }
        @Override public double getRoll() { return 2; }
        @Override public void resetGyro(double reset) { yaw = reset; }
    }

    private TestSwerve swerve() {
        return new TestSwerve(moduleConfig(0, 0, 0), moduleConfig(1, 0, 0), moduleConfig(2, 0, 0), moduleConfig(3, 0, 0));
    }

    @Test
    void drivingSendsEveryModuleItsState() {
        final TestSwerve swerve = swerve();
        swerve.fieldRelative = false;

        swerve.drive(new Translation2d(1, 0), 0.0);

        for (SwerveModule module : swerve.getModules()) {
            final VelocityVoltage drive = (VelocityVoltage) ((NAR_TalonFX) module.getDriveMotor()).getMotor().getAppliedControl();
            assertEquals(1 / 0.05, drive.Velocity, 1e-3);
        }
    }

    @Test
    void everyWayToDrive() {
        final TestSwerve swerve = swerve();
        swerve.drive(new Translation2d(1, 0), Rotation2d.fromDegrees(10));
        swerve.drive(new Translation2d(0, 1), 0.5);
        swerve.drive(1, 0, 0);
        swerve.drive(new ChassisSpeeds(0, 0, 1));

        // Throttle scales driving, not turning
        swerve.setThrottle(0.5);
        assertEquals(0.5, swerve.getThrottle(), TOLERANCE);
        swerve.toggleFieldRelative();
        swerve.drive(new ChassisSpeeds(2, 0, 0));
        final VelocityVoltage drive = (VelocityVoltage) ((NAR_TalonFX) swerve.getModule(0).getDriveMotor()).getMotor().getAppliedControl();
        assertEquals(1 / 0.05, drive.Velocity, 1e-3);

        swerve.setDriveVoltage(3);
        swerve.stop();
        swerve.setBrakeMode(false);
        assertEquals(Neutral.COAST, angleMotor(swerve.getModule(0)).neutralMode);
    }

    @Test
    void wheelLocksPointEachWheel() {
        final TestSwerve swerve = swerve();

        swerve.xLock();
        assertEquals(45 / (360 / 12.8), angleMotor(swerve.getModule(0)).lastPositionRotations, 1e-6);
        swerve.oLock();
        assertEquals(135 / (360 / 12.8), angleMotor(swerve.getModule(0)).lastPositionRotations, 1e-6);
        swerve.zeroLock();
        assertEquals(0, angleMotor(swerve.getModule(0)).lastPositionRotations, 1e-6);
        swerve.angleLock(20);
        assertEquals(20 / (360 / 12.8), angleMotor(swerve.getModule(3)).lastPositionRotations, 1e-6);
    }

    @Test
    void tracksWhereTheRobotIs() {
        final TestSwerve swerve = swerve();
        final Pose2d start = new Pose2d(2, 3, Rotation2d.fromDegrees(90));

        swerve.resetOdometry(start);
        swerve.periodic();

        assertEquals(2, swerve.getPose().getX(), 1e-3);
        assertEquals(3, swerve.getTranslation().getY(), 1e-3);
        assertEquals(90, swerve.getPose().getRotation().getDegrees(), 1e-3);
        assertEquals(90, swerve.getRawEstimatedPose().getRotation().getDegrees(), 1e-3);

        swerve.addVisionMeasurement(new Pose2d(2.1, 3, Rotation2d.fromDegrees(90)), Timer.getFPGATimestamp());
        swerve.resetAll();
        swerve.periodic();
        assertEquals(0, swerve.getPose().getX(), 1e-3);
    }

    @Test
    void velocitiesComeFromTheModules() {
        final TestSwerve swerve = swerve();
        assertEquals(4, swerve.getStates().length);
        assertEquals(4, swerve.getPositions().length);
        assertEquals(0, swerve.getSpeed(), 1e-3);
        assertEquals(0, swerve.getRobotVelocity().vxMetersPerSecond, 1e-3);
        assertEquals(0, swerve.getFieldVelocity().vxMetersPerSecond, 1e-3);
    }

    @Test
    void geometryHelpers() {
        final TestSwerve swerve = swerve();
        swerve.resetOdometry(new Pose2d(1, 1, Rotation2d.kZero));
        swerve.periodic();

        assertEquals(3, swerve.getDistanceTo(new Translation2d(4, 1)), 1e-3);
        assertEquals(-3, swerve.getTranslation2dTo(new Translation2d(4, 1)).getX(), 1e-3);
        assertEquals(new Translation2d(2, 1), swerve.nearestTranslation2d(List.of(new Translation2d(2, 1), new Translation2d(9, 9))));
        assertEquals(new Pose2d(2, 1, Rotation2d.kZero),
            swerve.nearestPose2d(List.of(new Pose2d(2, 1, Rotation2d.kZero), new Pose2d(9, 9, Rotation2d.kZero))));
        assertEquals(-90, swerve.getRotation2dTo(Rotation2d.fromDegrees(90)).getDegrees(), 1e-3);
        assertEquals(Math.toRadians(-90), swerve.getAngleTo(Rotation2d.fromDegrees(90)), 1e-3);
        assertTrue(Double.isFinite(swerve.getRotation2dTo(new Translation2d(1, 5)).getDegrees()));

        // One second at 1 m/s forward
        final Pose2d predicted = swerve.getPredictedPose(new ChassisSpeeds(1, 0, 0), 1);
        assertEquals(2, predicted.getX(), 1e-3);
    }

    @Test
    void modulesAreNumberedZeroToThree() {
        final TestSwerve swerve = swerve();
        assertSame(swerve.getModules()[2], swerve.getModule(2));
        assertThrows(IllegalArgumentException.class, () -> swerve.getModule(4));
    }

    @Test
    void toolsForSetup() {
        final TestSwerve swerve = swerve();
        swerve.initShuffleboard();
        // Prints each CANcoder's reading as a line of Java to paste into Constants
        swerve.identifyOffsetsCommand().initialize();
        assertNotNull(swerve.characterize(1, 4));
        assertEquals(1, swerve.getPitch(), TOLERANCE);
        assertEquals(2, swerve.getRoll(), TOLERANCE);
    }
}
