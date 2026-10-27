package common.hardware.limelight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import common.hardware.limelight.Limelight.LimelightKey;
import common.hardware.limelight.Limelight.Pipeline;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;

class LimelightTest {

    private static final double TOLERANCE = 1e-9;
    private static int nextName = 0;

    // A level camera 0.5 m up and 0.1 m behind the front of the robot
    private static final double CAMERA_HEIGHT = 0.5;
    private static final double FRONT_DISTANCE = 0.1;

    private Limelight limelight;
    private NetworkTable table;

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void createLimelight() {
        // A new table each test, so values don't carry over
        limelight = new Limelight("limelight-test" + nextName++, 0, CAMERA_HEIGHT, FRONT_DISTANCE, 0);
        table = limelight.limelightTable;
    }

    /** Sees a target 45 degrees up, straight ahead. */
    private void seeTarget() {
        table.getEntry(LimelightKey.VALID_TARGET.getKey()).setDouble(1);
        table.getEntry(LimelightKey.VERTICAL_OFFSET.getKey()).setDouble(45);
        table.getEntry(LimelightKey.HORIZONTAL_OFFSET.getKey()).setDouble(0);
    }

    @Test
    void readsValuesFromItsTable() {
        table.getEntry("ta").setDouble(2.5);
        assertEquals(2.5, limelight.getValue(LimelightKey.AREA), TOLERANCE);
        assertEquals(2.5, limelight.getValueAverage(LimelightKey.AREA, 5), TOLERANCE);
    }

    @Test
    void hasATargetOnlyWhenTheLimelightSaysSo() {
        assertFalse(limelight.hasValidTarget());
        seeTarget();
        assertTrue(limelight.hasValidTarget());
    }

    @Test
    void distanceToARaisedTarget() {
        seeTarget();
        // 2 m above the camera at 45 degrees is 2 m away from the camera, 1.9 m from the front
        assertEquals(2 - FRONT_DISTANCE, limelight.calculateDistToTopTarget(CAMERA_HEIGHT + 2), 1e-6);
    }

    @Test
    void targetPositionOnTheField() {
        seeTarget();
        final Pose2d robot = new Pose2d(1, 1, Rotation2d.fromDegrees(90));

        final Translation2d target = limelight.getTargetFieldPosition(CAMERA_HEIGHT + 2, robot);

        // 2 m straight ahead of a robot facing +y
        assertEquals(1, target.getX(), 1e-6);
        assertEquals(3, target.getY(), 1e-6);
    }

    @Test
    void noTargetGivesNoDistanceOrPosition() {
        assertEquals(-1, limelight.calculateDistToTopTarget(2), TOLERANCE);
        assertNull(limelight.getTargetFieldPosition(2, new Pose2d()));
    }

    @Test
    void pipelineIsWrittenToTheTable() {
        limelight.setPipeline(Pipeline.BLUE);
        assertEquals(Pipeline.BLUE.getPipeline(), limelight.getSelectedPipeline(), TOLERANCE);
    }
}
