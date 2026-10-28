package common.core.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import common.core.controllers.PIDFFConfig;
import common.core.subsystems.PivotMechanism.Orientation;
import common.hardware.motorcontroller.MockMotor;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;

class PivotMechanismTest {

    private static final double TOLERANCE = 1e-9;
    private static final DCMotor KRAKEN = DCMotor.getKrakenX60(1);

    // An arm on a 50:1 reduction, measured in degrees
    private static final double GEARING = 50;
    private static final double DEGREES_PER_MOTOR_ROTATION = 360 / GEARING;
    private static final double MIN = -90, MAX = 90, START = -90;

    private static final double KP = 0.1; // volts per degree
    private static final double KG = 0.5; // volts to hold the arm level

    private static int nextName = 0;

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    private static PivotMechanism arm(Orientation orientation, double degreesPerEncoderRotation, MockMotor motor) {
        final MotorConfig config = new MotorConfig(degreesPerEncoderRotation, 60, 40, false, Neutral.BRAKE);
        return new PivotMechanism("PivotMechanismTest" + nextName++, orientation, config, new PIDFFConfig(KP, 0, 0, 0, 0, 0, KG),
            1, MIN, MAX, START, Math.PI / 180, 0.5, 3, KRAKEN, motor);
    }

    private static PivotMechanism arm(double degreesPerEncoderRotation, MockMotor motor) {
        return arm(Orientation.FORWARD, degreesPerEncoderRotation, motor);
    }

    private static PivotMechanism arm(MockMotor motor) {
        return arm(DEGREES_PER_MOTOR_ROTATION, motor);
    }

    @Test
    void armSwingsUpToPointTheWayItsOrientationSays() {
        final Translation3d pivot = new Translation3d(0.1, -0.2, 0.5);
        final Translation3d[] directions = {
            new Translation3d(1, 0, 0), new Translation3d(-1, 0, 0), new Translation3d(0, 1, 0), new Translation3d(0, -1, 0)};
        final Orientation[] orientations = {Orientation.FORWARD, Orientation.BACKWARD, Orientation.LEFT, Orientation.RIGHT};

        for (int i = 0; i < orientations.length; i++) {
            final MockMotor motor = MockMotor.talonLike(1);
            final PivotMechanism arm = arm(orientations[i], DEGREES_PER_MOTOR_ROTATION, motor);
            arm.setPivot(pivot);

            // From hanging straight down at the start to level
            motor.rawPosition = 0;
            final Pose3d pose = arm.getComponentPose();

            // The tip of a 0.5 m arm starts below the pivot and ends up level with it
            final Translation3d tipAtStart = pivot.plus(new Translation3d(0, 0, -0.5));
            final Translation3d tip = tipAtStart.rotateBy(pose.getRotation()).plus(pose.getTranslation());
            assertEquals(0, tip.getDistance(pivot.plus(directions[i].times(0.5))), 1e-6, orientations[i].name());
        }
    }

    /** A turret on the same gearing, starting facing forward with no kG. */
    private static PivotMechanism turret(MockMotor motor) {
        final MotorConfig config = new MotorConfig(DEGREES_PER_MOTOR_ROTATION, 60, 40, false, Neutral.BRAKE);
        return new PivotMechanism("PivotMechanismTest" + nextName++, Orientation.TURRET, config, new PIDFFConfig(KP),
            1, -180, 180, 0, Math.PI / 180, 0.3, 5, KRAKEN, motor);
    }

    @Test
    void everyArmThatSwingsUpAndDownHasGravity() {
        for (Orientation orientation : new Orientation[] {Orientation.FORWARD, Orientation.BACKWARD, Orientation.LEFT, Orientation.RIGHT}) {
            final MockMotor motor = MockMotor.talonLike(1);
            arm(orientation, DEGREES_PER_MOTOR_ROTATION, motor);
            assertEquals(GEARING, motor.armRatio, TOLERANCE, orientation.name()); // kG follows the cosine
        }
    }

    /** A 0.2 m wrist on the same gearing, starting lined up with whatever it's mounted on. */
    private static PivotMechanism wrist(Orientation orientation, MockMotor motor) {
        final MotorConfig config = new MotorConfig(DEGREES_PER_MOTOR_ROTATION, 60, 40, false, Neutral.BRAKE);
        return new PivotMechanism("PivotMechanismTest" + nextName++, orientation, config,
            new PIDFFConfig(KP, 0, 0, 0, 0, 0, KG), 1, -90, 90, 0, Math.PI / 180, 0.2, 1, KRAKEN, motor);
    }

    @Test
    void wristOnAnArmWorksOutKGFromBothAnglesOnTheRoboRio() {
        final MockMotor armMotor = MockMotor.talonLike(1);
        final MockMotor wristMotor = MockMotor.talonLike(2);
        final PivotMechanism arm = arm(armMotor);
        final PivotMechanism wrist = wrist(Orientation.FORWARD, wristMotor);

        wrist.setParent(arm);

        // The motor controller no longer applies kG, since it only knows the wrist's angle
        assertEquals(0, wristMotor.nativeKG, TOLERANCE);
        assertEquals(0, wristMotor.armRatio, TOLERANCE);

        // Arm 30 degrees up, wrist another 30 degrees up from the arm
        armMotor.rawPosition = 30 / DEGREES_PER_MOTOR_ROTATION;
        wristMotor.rawPosition = 30 / DEGREES_PER_MOTOR_ROTATION;
        wrist.setSetpoint(30);
        wrist.periodic();

        assertEquals(Math.toRadians(60), wrist.getRadiansAboveLevel(), 1e-9);
        assertEquals(KG * Math.cos(Math.toRadians(60)), wristMotor.lastFeedforward, 1e-9);
    }

    @Test
    void wristPoseMovesWithTheArm() {
        final MockMotor armMotor = MockMotor.talonLike(1);
        final MockMotor wristMotor = MockMotor.talonLike(2);
        final PivotMechanism arm = arm(armMotor);
        final PivotMechanism wrist = wrist(Orientation.FORWARD, wristMotor);
        wrist.setParent(arm);

        // The 0.5 m arm hangs from 0.5 m up, so the wrist's pivot starts on the ground below it
        arm.setPivot(new Translation3d(0, 0, 0.5));
        wrist.setPivot(Translation3d.kZero);
        final Translation3d wristTipAtStart = new Translation3d(0, 0, -0.2);

        // Arm level: the wrist, still lined up with it, sticks straight out past its end
        armMotor.rawPosition = 0;
        assertEquals(0, tipOf(wrist, wristTipAtStart).getDistance(new Translation3d(0.7, 0, 0.5)), 1e-6);

        // Wrist turned 90 degrees up from the arm: it points straight up from the arm's end
        wristMotor.rawPosition = 90 / DEGREES_PER_MOTOR_ROTATION;
        assertEquals(0, tipOf(wrist, wristTipAtStart).getDistance(new Translation3d(0.5, 0, 0.7)), 1e-6);
    }

    /** Where a point on the mechanism, given at the starting position, is now. */
    private static Translation3d tipOf(PivotMechanism mechanism, Translation3d pointAtStart) {
        final Pose3d pose = mechanism.getComponentPose();
        return pointAtStart.rotateBy(pose.getRotation()).plus(pose.getTranslation());
    }

    /** An elevator moving straight up 1 cm per motor rotation, measured in meters. */
    private static LinearMechanism elevator(MockMotor motor) {
        final MotorConfig config = new MotorConfig(0.01, 60, 40, false, Neutral.BRAKE);
        return new LinearMechanism("PivotMechanismTest" + nextName++, config, new PIDFFConfig(KP), 0.01,
            0, 1, 1, Math.PI / 2, 5, KRAKEN, motor);
    }

    @Test
    void armOnAnElevatorRidesItUpAndKeepsItsGravityOnTheMotorController() {
        final MockMotor elevatorMotor = MockMotor.talonLike(1);
        final MockMotor armMotor = MockMotor.talonLike(2);
        final LinearMechanism elevator = elevator(elevatorMotor);
        final PivotMechanism arm = arm(armMotor);

        arm.setParent(elevator);
        arm.setPivot(new Translation3d(0, 0, 0.5));

        // Lifting doesn't tilt the arm, so its cosine kG still works
        assertEquals(GEARING, armMotor.armRatio, TOLERANCE);
        assertEquals(KG, armMotor.nativeKG, TOLERANCE);

        // Elevator up 0.3 m, arm level: the 0.5 m arm's tip is in front of its raised pivot
        elevatorMotor.rawPosition = 0.3 / 0.01;
        armMotor.rawPosition = 0;
        assertEquals(0, tipOf(arm, new Translation3d(0, 0, 0)).getDistance(new Translation3d(0.5, 0, 0.8)), 1e-6);
    }

    /** A telescope along the arm, which hangs straight down at the start, measured in meters. */
    private static LinearMechanism telescope(MockMotor motor) {
        final MotorConfig config = new MotorConfig(0.01, 60, 40, false, Neutral.BRAKE);
        return new LinearMechanism("PivotMechanismTest" + nextName++, config, new PIDFFConfig(KP, 0, 0, 0, 0, 0, KG),
            0.01, 0, 1, 1, -Math.PI / 2, 2, KRAKEN, motor);
    }

    @Test
    void telescopeOnAnArmWorksOutKGFromItsInclineOnTheRoboRio() {
        final MockMotor armMotor = MockMotor.talonLike(1);
        final MockMotor telescopeMotor = MockMotor.talonLike(2);
        final PivotMechanism arm = arm(armMotor);
        final LinearMechanism telescope = telescope(telescopeMotor);

        telescope.setParent(arm);

        // The motor controller no longer applies kG, since it can't see the arm's angle
        assertEquals(0, telescopeMotor.nativeKG, TOLERANCE);

        // Arm 30 degrees above level: half the telescope's weight pulls along it
        armMotor.rawPosition = 30 / DEGREES_PER_MOTOR_ROTATION;
        telescope.setSetpoint(0.2);
        telescope.periodic();
        assertEquals(0.5, telescope.getInclineSine(), 1e-9);
        assertEquals(KG * 0.5, telescopeMotor.lastFeedforward, 1e-9);

        // Arm level: none of it does
        armMotor.rawPosition = 0;
        assertEquals(0, telescope.getInclineSine(), 1e-9);
    }

    @Test
    void wristOnATelescopeSwingsWithTheArmWhateverOrderTheyreMounted() {
        final MockMotor armMotor = MockMotor.talonLike(1);
        final MockMotor wristMotor = MockMotor.talonLike(3);
        final PivotMechanism arm = arm(armMotor);
        final LinearMechanism telescope = telescope(MockMotor.talonLike(2));
        final PivotMechanism wrist = wrist(Orientation.FORWARD, wristMotor);

        // Wrist first, before the telescope is on the arm
        wrist.setParent(telescope);
        assertEquals(KG, wristMotor.nativeKG, TOLERANCE); // nothing tilts it yet
        telescope.setParent(arm);

        assertEquals(0, wristMotor.nativeKG, TOLERANCE);
        armMotor.rawPosition = 30 / DEGREES_PER_MOTOR_ROTATION;
        wristMotor.rawPosition = 20 / DEGREES_PER_MOTOR_ROTATION;
        assertEquals(Math.toRadians(50), wrist.getRadiansAboveLevel(), 1e-9);
    }

    @Test
    void wristMustSwingTheSameWayAsItsArm() {
        final PivotMechanism arm = arm(MockMotor.talonLike(1));
        final PivotMechanism wrist = wrist(Orientation.LEFT, MockMotor.talonLike(2));
        assertThrows(IllegalArgumentException.class, () -> wrist.setParent(arm));
    }

    @Test
    void hoodOnATurretKeepsItsGravityOnTheMotorController() {
        final PivotMechanism turret = turret(MockMotor.talonLike(1));
        final MockMotor hoodMotor = MockMotor.talonLike(2);
        final PivotMechanism hood = wrist(Orientation.FORWARD, hoodMotor);

        hood.setParent(turret);

        assertEquals(GEARING, hoodMotor.armRatio, TOLERANCE);
        assertEquals(KG, hoodMotor.nativeKG, TOLERANCE);
    }

    @Test
    void turretHasNoGravityFeedforward() {
        final MockMotor motor = MockMotor.talonLike(1);
        turret(motor);
        assertEquals(0, motor.armRatio, TOLERANCE); // kG isn't multiplied by a cosine
        assertEquals(0, motor.nativeKG, TOLERANCE);
    }

    @Test
    void turretPoseTurnsAboutThePivot() {
        final MockMotor motor = MockMotor.talonLike(1);
        final PivotMechanism turret = turret(motor);
        final Translation3d pivot = new Translation3d(-0.1, 0.05, 0.4);
        turret.setPivot(pivot);

        // From facing forward to 90 degrees counterclockwise, facing left
        motor.rawPosition = 90 / DEGREES_PER_MOTOR_ROTATION;
        final Pose3d pose = turret.getComponentPose();

        // The end of the barrel, 0.3 m in front of the pivot, ends up 0.3 m to its left
        final Translation3d barrelAtStart = pivot.plus(new Translation3d(0.3, 0, 0));
        final Translation3d barrel = barrelAtStart.rotateBy(pose.getRotation()).plus(pose.getTranslation());
        assertEquals(0, barrel.getDistance(pivot.plus(new Translation3d(0, 0.3, 0))), 1e-6);
    }

    @Test
    void poseRotatesAboutThePivot() {
        final MockMotor motor = MockMotor.talonLike(1);
        final PivotMechanism arm = arm(motor);
        final Translation3d pivot = new Translation3d(0.2, 0, 0.5);
        arm.setPivot(pivot);

        // From hanging straight down at the start to level, pointing forward
        motor.rawPosition = 0;
        final Pose3d pose = arm.getComponentPose();

        // The tip of a 0.5 m arm starts below the pivot and ends up in front of it
        final Translation3d tipAtStart = pivot.plus(new Translation3d(0, 0, -0.5));
        final Translation3d tip = tipAtStart.rotateBy(pose.getRotation()).plus(pose.getTranslation());
        assertEquals(0.7, tip.getX(), 1e-6);
        assertEquals(0, tip.getY(), 1e-6);
        assertEquals(0.5, tip.getZ(), 1e-6);

        // The pivot itself doesn't move
        final Translation3d movedPivot = pivot.rotateBy(pose.getRotation()).plus(pose.getTranslation());
        assertEquals(0, movedPivot.getDistance(pivot), 1e-6);
    }

    @Test
    void gravityFollowsTheArmsCosineInArmRotations() {
        final MockMotor motor = MockMotor.talonLike(1);
        arm(motor);

        assertEquals(GEARING, motor.armRatio, TOLERANCE);
        assertEquals(GEARING, motor.mechanismRatio, TOLERANCE);
        assertEquals(KG, motor.nativeKG, TOLERANCE);
        // 0.1 volts per degree is 36 volts per arm rotation
        assertEquals(KP * 360, motor.nativeKP, TOLERANCE);
    }

    @Test
    void encoderStartsAtTheStartPosition() {
        final MockMotor motor = MockMotor.talonLike(1);
        final PivotMechanism arm = arm(motor);
        assertEquals(START, arm.getPosition(), TOLERANCE);
    }

    @Test
    void setpointIsSentInMotorRotations() {
        final MockMotor motor = MockMotor.talonLike(1);
        final PivotMechanism arm = arm(motor);

        arm.setSetpoint(45);
        arm.periodic();

        assertEquals(Control.Position, motor.lastMode);
        assertEquals(45 / DEGREES_PER_MOTOR_ROTATION, motor.lastPositionRotations, TOLERANCE);
    }

    @Test
    void setpointIsLimitedToTheArmsTravel() {
        final PivotMechanism arm = arm(MockMotor.talonLike(1));
        arm.setSetpoint(180);
        assertEquals(MAX, arm.getSetpoint(), TOLERANCE);
        arm.setSetpoint(-180);
        assertEquals(MIN, arm.getSetpoint(), TOLERANCE);
    }

    @Test
    void armKeepsHoldingAgainstGravityAtSetpoint() {
        final MockMotor motor = MockMotor.talonLike(1);
        final PivotMechanism arm = arm(motor);
        arm.setSetpoint(START);

        arm.periodic();

        assertTrue(arm.isEnabled());
    }

    @Test
    void encoderOnTheArmShaftGivesAOneToOneGravityRatio() {
        final MockMotor motor = MockMotor.sparkLike(1);
        motor.setMotorRotationsPerSensorRotation(GEARING); // the encoder turns once per arm rotation
        arm(360, motor); // 360 degrees per encoder rotation

        assertEquals(1, motor.armRatio, TOLERANCE);
    }
}
