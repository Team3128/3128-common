package common.hardware.camera;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Represents and holds the specs of a camera.
 * 
 * @since 2022 Rapid React
 * @author Mason Lam, William Yuan
 */
public class Camera {

    public final String name;

    public final double cameraPitch;

    public final double xOffset;

    public final double yOffset;

    public final double pitchOffset;

    public final double yawOffset;

    public final AprilTagFields aprilTags;

    public final PoseStrategy calc_strategy;

    public final PhotonPoseEstimator estimator;

    public Camera(String name, double cameraPitch, double xOffset, double yOffset, double pitchOffset, double yawOffset,
            AprilTagFields aprilTags, PoseStrategy calc_strategy) {
        this.name = name;
        this.cameraPitch = cameraPitch;
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.pitchOffset = pitchOffset;
        this.yawOffset = yawOffset;
        this.aprilTags = aprilTags;
        this.calc_strategy = calc_strategy;

        Transform3d offset = new Transform3d(xOffset, yOffset, 0, 
        new Rotation3d(0.0, pitchOffset, 0.0).rotateBy(new Rotation3d(0.0,0.0,yawOffset))
        );

        estimator = new PhotonPoseEstimator(aprilTags.loadAprilTagLayoutField(), 
        calc_strategy,
        new PhotonCamera(name), offset
        );

        estimator.setReferencePose(new Pose2d());
    }

}