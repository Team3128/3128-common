package common.hardware.camera;

import java.util.LinkedList;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Team 3128's class to control the robot's cameras and vision processing.
 * 
 * @since 2022 Rapid React
 * @author Mason Lam, William Yuan
 */
public class Camera {

    private final PhotonCamera camera;
    private final Transform3d offset;
    private final PhotonPoseEstimator estimator;

    private boolean isDisabled = false;
    private PhotonPipelineResult lastResult;

    private static AprilTagFieldLayout aprilTags;
    private static PoseStrategy calc_strategy;
    private static BiConsumer<Pose2d, Double> odometry;
    private static Supplier<Pose2d> robotPose;

    private static final LinkedList<Camera> cameras = new LinkedList<Camera>();
    
    public Camera(String name, double xOffset, double yOffset, double yawOffset, double pitchOffset, double rollOffset) {
        camera = new PhotonCamera(name);

        this.offset = new Transform3d(xOffset, yOffset, 0, 
            new Rotation3d(0.0, pitchOffset, 0.0)
            .rotateBy(new Rotation3d(rollOffset, 0, 0))
            .rotateBy(new Rotation3d(0.0,0.0,yawOffset))
        );

        if (aprilTags == null || calc_strategy == null || odometry == null || robotPose == null) {
            throw new IllegalStateException("Camera not configured");
        }

        estimator = new PhotonPoseEstimator(
            aprilTags, 
            calc_strategy,
            camera,
            offset
        );

        cameras.add(this);
    }

    public static void configCameras(AprilTagFields aprilTagLayout, PoseStrategy calc_strategy, BiConsumer<Pose2d, Double> odometry, Supplier<Pose2d> robotPose){
        Camera.aprilTags = aprilTagLayout.loadAprilTagLayoutField();
        Camera.calc_strategy = calc_strategy;
        Camera.odometry = odometry;
        Camera.robotPose = robotPose;
    }

    public static void updateAll(){
        for (final Camera camera : cameras) {
            camera.update();
        }
    }   

    public void update(){
        if (isDisabled) return;
        lastResult = camera.getLatestResult();
        if (!lastResult.hasTargets()) {
            Logger.recordOutput("Vision/" + camera.getName(), robotPose.get());
            return;
        }

        if (estimator.getPrimaryStrategy() == PoseStrategy.CLOSEST_TO_REFERENCE_POSE) {
            estimator.setReferencePose(robotPose.get());
        }

        final Optional<EstimatedRobotPose> estimatedPose = estimator.update(lastResult);

        if(!estimatedPose.isPresent()) {
            Logger.recordOutput("Vision/" + camera.getName(), robotPose.get());
        }

        final EstimatedRobotPose pose = estimatedPose.get();
        //To do add logging for this
        odometry.accept(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
    }

    public PhotonPipelineResult getLatestResult() {
        return lastResult;
    }

    public Transform3d getOffset() {
        return offset;
    }

    public void disable(){
        isDisabled = true;
    }

    public void enable() {
        isDisabled = false;
    }

}