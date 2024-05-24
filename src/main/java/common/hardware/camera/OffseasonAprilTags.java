package common.hardware.camera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

import java.util.HashMap;

public class OffseasonAprilTags {
    public static HashMap<Integer, Pose3d> offSeasonTagMap = new HashMap<Integer, Pose3d>();
    static {
        offSeasonTagMap.put(1, new Pose3d(
            Units.inchesToMeters(593.68),
            Units.inchesToMeters(9.68),
            Units.inchesToMeters(53.38),
            new Rotation3d(0,0,Units.degreesToRadians(120))
        ));
        offSeasonTagMap.put(2, new Pose3d(
            Units.inchesToMeters(637.21),
            Units.inchesToMeters(34.79),
            Units.inchesToMeters(53.38),
            new Rotation3d(0,0,Units.degreesToRadians(120))
        ));
        offSeasonTagMap.put(3, new Pose3d(
            Units.inchesToMeters(652.73),
            Units.inchesToMeters(196.17),
            Units.inchesToMeters(57.13),
            new Rotation3d(0,0,Units.degreesToRadians(180))
        ));
        offSeasonTagMap.put(4, new Pose3d(
            Units.inchesToMeters(652.73),
            Units.inchesToMeters(218.42),
            Units.inchesToMeters(57.13),
            new Rotation3d(0,0,Units.degreesToRadians(180))
        ));        
        offSeasonTagMap.put(5, new Pose3d(
            Units.inchesToMeters(578.77),
            Units.inchesToMeters(323),
            Units.inchesToMeters(53.38),
            new Rotation3d(0,0,Units.degreesToRadians(270))
        ));
        offSeasonTagMap.put(6, new Pose3d(
            Units.inchesToMeters(72.5),
            Units.inchesToMeters(323),
            Units.inchesToMeters(53.38),
            new Rotation3d(0,0,Units.degreesToRadians(270))
        ));
        offSeasonTagMap.put(7, new Pose3d(
            Units.inchesToMeters(-1.5),
            Units.inchesToMeters(218.42),
            Units.inchesToMeters(57.13),
            new Rotation3d(0,0,Units.degreesToRadians(0))
        ));
        offSeasonTagMap.put(8, new Pose3d(
            Units.inchesToMeters(-1.5),
            Units.inchesToMeters(196.17),
            Units.inchesToMeters(57.13),
            new Rotation3d(0,0,Units.degreesToRadians(0))
        ));
        offSeasonTagMap.put(9, new Pose3d(
            Units.inchesToMeters(14.02),
            Units.inchesToMeters(34.79),
            Units.inchesToMeters(53.38),
            new Rotation3d(0,0,Units.degreesToRadians(60))
        ));
        offSeasonTagMap.put(10, new Pose3d(
            Units.inchesToMeters(57.54),
            Units.inchesToMeters(9.68),
            Units.inchesToMeters(53.38),
            new Rotation3d(0,0,Units.degreesToRadians(60))
        ));
        offSeasonTagMap.put(11, new Pose3d(
            Units.inchesToMeters(468.69),
            Units.inchesToMeters(146.19),
            Units.inchesToMeters(52),
            new Rotation3d(0,0,Units.degreesToRadians(300))
        ));
        offSeasonTagMap.put(12, new Pose3d(
            Units.inchesToMeters(468.69),
            Units.inchesToMeters(177.1),
            Units.inchesToMeters(52),
            new Rotation3d(0,0,Units.degreesToRadians(60))
        ));
        offSeasonTagMap.put(13, new Pose3d(
            Units.inchesToMeters(441.74),
            Units.inchesToMeters(161.62),
            Units.inchesToMeters(52),
            new Rotation3d(0,0,Units.degreesToRadians(180))
        ));
        offSeasonTagMap.put(14, new Pose3d(
            Units.inchesToMeters(209.48),
            Units.inchesToMeters(161.62),
            Units.inchesToMeters(52),
            new Rotation3d(0,0,Units.degreesToRadians(0))
        ));
        offSeasonTagMap.put(15, new Pose3d(
            Units.inchesToMeters(182.73),
            Units.inchesToMeters(177.1),
            Units.inchesToMeters(52),
            new Rotation3d(0,0,Units.degreesToRadians(120))
        ));
        offSeasonTagMap.put(16, new Pose3d(
            Units.inchesToMeters(182.73),
            Units.inchesToMeters(146.19),
            Units.inchesToMeters(52),
            new Rotation3d(0,0,Units.degreesToRadians(240))
        ));
    }
}
