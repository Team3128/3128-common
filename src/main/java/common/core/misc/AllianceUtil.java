package common.core.misc;

import java.util.Optional;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Reads the robot's alliance and converts field positions measured from the blue alliance to the
 * robot's own alliance.
 *
 * <p>The alliance is read from the Driver Station on every call rather than stored, because it can
 * change while the robot code runs, for example when the robot connects to the FMS or the alliance
 * is switched in the Driver Station or simulator.
 *
 * <p>Flipping uses PathPlanner's {@link FlippingUtil}, so positions are flipped the same way as
 * paths. The robot project sets {@link FlippingUtil#symmetryType} once at startup to match the
 * season's field.
 */
public final class AllianceUtil {

    private AllianceUtil() {}

    /**
     * Returns the alliance reported by the Driver Station, or an empty {@code Optional} if the
     * Driver Station has not reported one yet.
     */
    public static Optional<Alliance> getAlliance() {
        return DriverStation.getAlliance();
    }

    /** Returns whether the robot is on the red alliance. An unknown alliance counts as blue. */
    public static boolean isRed() {
        return getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    /**
     * Converts a position measured from the blue alliance to the robot's alliance.
     *
     * @param bluePosition The position on the blue side of the field.
     * @return The position unchanged on the blue alliance, or flipped to the red side on the red
     *     alliance.
     */
    public static Translation2d flip(Translation2d bluePosition) {
        return isRed() ? FlippingUtil.flipFieldPosition(bluePosition) : bluePosition;
    }

    /**
     * Converts a pose measured from the blue alliance to the robot's alliance.
     *
     * @param bluePose The pose on the blue side of the field.
     * @return The pose unchanged on the blue alliance, or flipped to the red side on the red
     *     alliance.
     */
    public static Pose2d flip(Pose2d bluePose) {
        return isRed() ? FlippingUtil.flipFieldPose(bluePose) : bluePose;
    }
}
