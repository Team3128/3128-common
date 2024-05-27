package common.hardware.camera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

import java.util.HashMap;

public class OffseasonAprilTags {
    public static HashMap<Integer, Pose3d> offSeasonTagMap = new HashMap<Integer, Pose3d>();
    static {
        offSeasonTagMap.put(1, new Pose3d(
            13.2173,
            0.2458,
            1.3558,
            new Rotation3d(0,0,Units.degreesToRadians(120))
        ));
        offSeasonTagMap.put(2, new Pose3d(
            14.322,
            0.8836,
            1.3558,
            new Rotation3d(0,0,Units.degreesToRadians(120))
        ));
        offSeasonTagMap.put(3, new Pose3d(
            14.7172,
            4.9827,
            1.4511,
            new Rotation3d(0,0,Units.degreesToRadians(180))
        ));
        offSeasonTagMap.put(4, new Pose3d(
            14.7172,
            5.5478,
            1.4511,
            new Rotation3d(0,0,Units.degreesToRadians(180))
        ));        
        offSeasonTagMap.put(5, new Pose3d(
            12.8386,
            8.2042,
            1.3558,
            new Rotation3d(0,0,Units.degreesToRadians(270))
        ));
        offSeasonTagMap.put(6, new Pose3d(
            1.8415,
            8.2042,
            1.3558,
            new Rotation3d(0,0,Units.degreesToRadians(270))
        ));
        offSeasonTagMap.put(7, new Pose3d(
            -0.03809,
            5.5478,
            1.4511,
            new Rotation3d(0,0,Units.degreesToRadians(0))
        ));
        offSeasonTagMap.put(8, new Pose3d(
            -0.0380,
            4.9827,
            1.4511,
            new Rotation3d(0,0,Units.degreesToRadians(0))
        ));
        offSeasonTagMap.put(9, new Pose3d(
            0.3561,
            0.8836,
            1.3558,
            new Rotation3d(0,0,Units.degreesToRadians(60))
        ));
        offSeasonTagMap.put(10, new Pose3d(
            1.4615,
            0.2458,
            1.3558,
            new Rotation3d(0,0,Units.degreesToRadians(60))
        ));
        offSeasonTagMap.put(11, new Pose3d(
            10.0425,
            3.7132,
            1.3208,
            new Rotation3d(0,0,Units.degreesToRadians(300))
        ));
        offSeasonTagMap.put(12, new Pose3d(
            10.0425,
            4.4983,
            1.3208,
            new Rotation3d(0,0,Units.degreesToRadians(60))
        ));
        offSeasonTagMap.put(13, new Pose3d(
            9.3580,
            4.1051,
            1.3208,
            new Rotation3d(0,0,Units.degreesToRadians(180))
        ));
        offSeasonTagMap.put(14, new Pose3d(
            5.3207,
            4.1051,
            1.3208,
            new Rotation3d(0,0,Units.degreesToRadians(0))
        ));
        offSeasonTagMap.put(15, new Pose3d(
            4.6413,
            4.4983,
            1.3208,
            new Rotation3d(0,0,Units.degreesToRadians(120))
        ));
        offSeasonTagMap.put(16, new Pose3d(
            4.6413,
            3.7132,
            1.3208,
            new Rotation3d(0,0,Units.degreesToRadians(240))
        ));
    }
}
