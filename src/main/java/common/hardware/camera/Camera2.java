package common.hardware.camera;

import java.util.Optional;
import java.util.function.BiConsumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class Camera2 {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final BiConsumer<Pose2d, Double> estConsumer;
    private boolean enabled = true;
    private double minDistThreshold = 0, maxDistThreshold = 100, ambiguityThreshold = 0.2;

    public Camera2(String cameraName, Transform3d robotToCam, AprilTagFieldLayout tagLayout, BiConsumer<Pose2d, Double> estConsumer) {
        camera = new PhotonCamera(cameraName);
        poseEstimator = new PhotonPoseEstimator(tagLayout, robotToCam);
        this.estConsumer = estConsumer;
        initShuffleboard();
    }

    public void initShuffleboard() {
        NAR_Shuffleboard.addData(camera.getName(), "Enabled", () -> enabled, 0, 0);
    }

    public void periodic() {
        if (!enabled) return;
        Optional<EstimatedRobotPose> curEst = Optional.empty();
        for (var result : camera.getAllUnreadResults()) {
            for (int i = 0; i < result.targets.size(); i++) {
                if (result.targets.get(i).poseAmbiguity > ambiguityThreshold ||
                    result.targets.get(i).bestCameraToTarget.getTranslation().getNorm() < minDistThreshold ||
                    result.targets.get(i).bestCameraToTarget.getTranslation().getNorm() > maxDistThreshold) {
                        result.targets.remove(i);
                        i--;
                }
            }
            curEst = poseEstimator.estimateCoprocMultiTagPose(result);
            if (curEst.isEmpty()) {
                curEst = poseEstimator.estimateLowestAmbiguityPose(result);
            }
            curEst.ifPresent(
                est -> {
                    estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds);
                }
            );
        }
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    // public void setThresholds(double minDistThreshold, double maxDistThreshold) {
    //     this.minDistThreshold = minDistThreshold;
    //     this.maxDistThreshold = maxDistThreshold;
    // }

    public void setThresholds(double minDistThreshold, double maxDistThreshold, double ambiguityThreshold) {
        this.minDistThreshold = minDistThreshold;
        this.ambiguityThreshold = ambiguityThreshold;
        this.maxDistThreshold = maxDistThreshold;
    }
}
