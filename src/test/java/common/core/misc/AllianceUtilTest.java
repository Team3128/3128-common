package common.core.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.FlippingUtil.FieldSymmetry;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

class AllianceUtilTest {

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
        FlippingUtil.symmetryType = FieldSymmetry.kRotational;
    }

    @AfterEach
    void resetAlliance() {
        setStation(AllianceStationID.Unknown);
    }

    private static void setStation(AllianceStationID station) {
        DriverStationSim.setAllianceStationId(station);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
    }

    @Test
    void unknownAllianceCountsAsBlue() {
        setStation(AllianceStationID.Unknown);
        assertEquals(Optional.empty(), AllianceUtil.getAlliance());
        assertFalse(AllianceUtil.isRed());
    }

    @Test
    void followsTheDriverStationWhenTheAllianceChanges() {
        setStation(AllianceStationID.Blue1);
        assertEquals(Optional.of(Alliance.Blue), AllianceUtil.getAlliance());
        assertFalse(AllianceUtil.isRed());

        setStation(AllianceStationID.Red2);
        assertEquals(Optional.of(Alliance.Red), AllianceUtil.getAlliance());
        assertTrue(AllianceUtil.isRed());
    }

    @Test
    void leavesPositionsAloneOnBlue() {
        setStation(AllianceStationID.Blue1);
        final Translation2d position = new Translation2d(1, 2);
        final Pose2d pose = new Pose2d(position, Rotation2d.fromDegrees(30));
        assertEquals(position, AllianceUtil.flip(position));
        assertEquals(pose, AllianceUtil.flip(pose));
    }

    @Test
    void flipsPositionsOnRed() {
        setStation(AllianceStationID.Red1);
        final Translation2d position = new Translation2d(1, 2);
        final Pose2d pose = new Pose2d(position, Rotation2d.fromDegrees(30));
        assertEquals(FlippingUtil.flipFieldPosition(position), AllianceUtil.flip(position));
        assertEquals(FlippingUtil.flipFieldPose(pose), AllianceUtil.flip(pose));
    }
}
