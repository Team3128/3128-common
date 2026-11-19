package common.hardware.camera;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.simulation.SimHooks;

/** Checks that a simulated camera sees the field's AprilTags and estimates where the robot is. */
class DynamicCameraTest {

    private static final AprilTagFieldLayout FIELD = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    private static int nextName = 0;
    /** The camera from the last {@code estimatesFrom}, to check what it shows. */
    private static DynamicCamera lastCamera;

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @AfterEach
    void restoreTime() {
        SimHooks.resumeTiming();
    }

    /** Runs the vision sim and the cameras for a second, with the robot at {@code robotPose}. */
    private static List<Pose2d> estimatesFrom(Pose2d robotPose) {
        return estimatesFrom(robotPose, camera -> camera.setThresholds(0, 10, 1));
    }

    /** Like {@link #estimatesFrom(Pose2d)}, with the camera set up by {@code setup}. */
    private static List<Pose2d> estimatesFrom(Pose2d robotPose, Consumer<DynamicCamera> setup) {
        final List<Pose2d> estimates = new ArrayList<>();
        final DynamicCamera camera = new DynamicCamera("DynamicCameraTest" + nextName++,
            // Level with the tag, at the robot's center
            new Transform3d(0, 0, FIELD.getTags().get(0).pose.getZ(), new Rotation3d()),
            FIELD, (pose, time) -> estimates.add(pose));
        setup.accept(camera);
        lastCamera = camera;

        SimHooks.pauseTiming();
        for (int i = 0; i < 50; i++) {
            SimHooks.stepTiming(0.02);
            DynamicCamera.updateSimulation(robotPose);
            DynamicCamera.periodic();
        }
        DynamicCamera.dynamicCameras.remove(camera);
        return estimates;
    }

    @Test
    void cameraFacingATagEstimatesWhereTheRobotIs() {
        // 2 m out from the first tag, facing it
        final AprilTag tag = FIELD.getTags().get(0);
        final Pose2d robotPose = tag.pose.toPose2d().transformBy(new Transform2d(2, 0, Rotation2d.k180deg));

        final List<Pose2d> estimates = estimatesFrom(robotPose);

        assertFalse(estimates.isEmpty(), "the camera should see the tag");
        final Pose2d last = estimates.get(estimates.size() - 1);
        assertEquals(0, last.getTranslation().getDistance(robotPose.getTranslation()), 0.15);
        assertEquals(0, last.getRotation().minus(robotPose.getRotation()).getDegrees(), 5);

        // It shows what it saw
        assertEquals(last, lastCamera.estimatedPose);
        assertTrue(lastCamera.withinDist);
        assertEquals(2, lastCamera.distance, 0.15);
    }

    @Test
    void pictureWithEveryTagFartherThanTheLimitIsIgnored() {
        // 2 m from the first tag, where it sees it and its neighbors
        final AprilTag tag = FIELD.getTags().get(0);
        final Pose2d robotPose = tag.pose.toPose2d().transformBy(new Transform2d(2, 0, Rotation2d.k180deg));

        // Only trusts tags within 1.5 m, and they're all farther
        final List<Pose2d> estimates = estimatesFrom(robotPose, camera -> camera.setThresholds(0, 1.5, 1));

        assertTrue(estimates.isEmpty());
        assertFalse(lastCamera.withinDist);
    }

    @Test
    void disabledCameraIsIgnored() {
        final AprilTag tag = FIELD.getTags().get(0);
        final Pose2d robotPose = tag.pose.toPose2d().transformBy(new Transform2d(2, 0, Rotation2d.k180deg));

        final List<Pose2d> estimates = estimatesFrom(robotPose, camera -> {
            camera.setThresholds(0, 10, 1);
            camera.isDisabled = true;
        });

        assertTrue(estimates.isEmpty());
    }

    @Test
    void turnedAwayFromATagAnythingElseItSeesStillPlacesTheRobot() {
        // Same spot, turned around so the tag is behind it
        final AprilTag tag = FIELD.getTags().get(0);
        final Pose2d robotPose = tag.pose.toPose2d().transformBy(new Transform2d(2, 0, Rotation2d.kZero));

        // Other tags may be in view from here
        final List<Pose2d> estimates = estimatesFrom(robotPose);
        for (Pose2d estimate : estimates) {
            assertTrue(estimate.getTranslation().getDistance(robotPose.getTranslation()) < 0.5,
                "anything it does see should still put the robot in the right place");
        }
    }

    @Test
    void turretCameraLooksWhereTheTurretPoints() {
        // The robot faces away from the tag, but its turret is turned around to face it
        final AprilTag tag = FIELD.getTags().get(0);
        final Pose2d robotPose = tag.pose.toPose2d().transformBy(new Transform2d(2, 0, Rotation2d.kZero));
        final List<Pose2d> estimates = new ArrayList<>();
        final DynamicCamera camera = new DynamicCamera("DynamicCameraTest" + nextName++,
            Transform3d.kZero, FIELD, (pose, time) -> estimates.add(pose),
            new Translation3d(0, 0, tag.pose.getZ()), () -> Rotation2d.k180deg);
        camera.setThresholds(0, 10, 0.2);

        SimHooks.pauseTiming();
        for (int i = 0; i < 50; i++) {
            SimHooks.stepTiming(0.02);
            DynamicCamera.updateSimulation(robotPose);
            DynamicCamera.periodic();
        }
        DynamicCamera.dynamicCameras.remove(camera);

        assertFalse(estimates.isEmpty(), "the turret camera should see the tag behind the robot");
        final Pose2d last = estimates.get(estimates.size() - 1);
        assertEquals(0, last.getTranslation().getDistance(robotPose.getTranslation()), 0.15);
        assertDoesNotThrow(camera::initShuffleboard);
    }

    @Test
    void visionCanBeTurnedOff() {
        final AprilTag tag = FIELD.getTags().get(0);
        final Pose2d robotPose = tag.pose.toPose2d().transformBy(new Transform2d(2, 0, Rotation2d.k180deg));

        DynamicCamera.disable();
        try {
            assertTrue(estimatesFrom(robotPose).isEmpty());
        } finally {
            DynamicCamera.enable();
        }
    }
}
