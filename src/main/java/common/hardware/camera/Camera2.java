package common.hardware.camera;

import java.util.Optional;
import java.util.function.BiConsumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class Camera2 {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final BiConsumer<Pose2d, Double> estConsumer;

    public Camera2(String cameraName, Transform3d robotToCam, AprilTagFieldLayout tagLayout, BiConsumer<Pose2d, Double> estConsumer) {
        camera = new PhotonCamera(cameraName);
        poseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.AVERAGE_BEST_TARGETS, robotToCam);
        this.estConsumer = estConsumer;
    }

    public void periodic() {
        Optional<EstimatedRobotPose> curEst = Optional.empty();
        for (var result : camera.getAllUnreadResults()) {
            curEst = poseEstimator.update(result);
            curEst.ifPresent(
                est -> {
                    estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds);
                }
            );
        }
    }
}
