package common.hardware.camera;

import java.util.Optional;
import java.util.function.BiConsumer;

import org.photonvision.EstimatedRobotPose;
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

    private final String name;

    private final double xOffset;

    private final double yOffset;

    private final double pitchOffset;

    private final double yawOffset;

    private final AprilTagFields aprilTags;

    private final PoseStrategy calc_strategy;

    private final PhotonPoseEstimator estimator;
    private EstimatedRobotPose lastEstimatedPose = null;

    private final BiConsumer<Pose2d, Double> odometry;
    
    public Camera(String name, double xOffset, double yOffset, double pitchOffset, double yawOffset,
            AprilTagFields aprilTags, PoseStrategy calc_strategy, BiConsumer<Pose2d, Double> odometry) {
        this.name = name;
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.pitchOffset = pitchOffset;
        this.yawOffset = yawOffset;
        this.aprilTags = aprilTags;
        this.calc_strategy = calc_strategy;
        this.odometry = odometry;

        Transform3d offset = new Transform3d(xOffset, yOffset, 0, 
            new Rotation3d(0.0, pitchOffset, 0.0).rotateBy(new Rotation3d(0.0,0.0,yawOffset))
        );

        estimator = new PhotonPoseEstimator(
            aprilTags.loadAprilTagLayoutField(), 
            calc_strategy,
            new PhotonCamera(name),
            offset
        );

        estimator.setReferencePose(new Pose2d());
    }

    public void update(){
        if (lastEstimatedPose != null)
            estimator.setReferencePose(lastEstimatedPose.estimatedPose.toPose2d());
        Optional<EstimatedRobotPose> estimatedPose = estimator.update();

        if(!estimatedPose.isPresent()) return;
        EstimatedRobotPose pose = estimatedPose.get();
        odometry.accept(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
        lastEstimatedPose = pose;
    }

    public EstimatedRobotPose getLastEstimatedPose(){
        return lastEstimatedPose;
    }

}