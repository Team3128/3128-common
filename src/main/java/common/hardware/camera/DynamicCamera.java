package common.hardware.camera;

import java.util.LinkedList;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Function;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class DynamicCamera {

    //static variables
    public static final LinkedList<DynamicCamera> dynamicCameras = new LinkedList<DynamicCamera>();
    public static boolean enabled = true;

    //dynamic camera related variables
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator poseEstimator;
    private final BiConsumer<Pose2d, Double> estConsumer;
    private final boolean isDynamic;
    // The camera's fixed mount offset off of whatever it's bolted to (the turret pivot for a
    // dynamic camera, the chassis for a static one).
    private final Transform3d pivotToCamera;
    private Translation2d offset;
    private Function<Double, Rotation2d> angularOffset;
    private double minDistThreshold = 0, maxDistThreshold = 100, ambiguityThreshold = 0.2;

    // estConsumer sends estimated poses to the SwerveBase where they are factored into the robot's odometry
    //not dynamic constructor
    public DynamicCamera(String cameraName, Transform3d robotToCam, AprilTagFieldLayout tagLayout, BiConsumer<Pose2d, Double> estConsumer) {
        this.photonCamera = new PhotonCamera(cameraName);
        this.poseEstimator = new PhotonPoseEstimator(tagLayout, robotToCam);
        this.estConsumer = estConsumer;
        this.isDynamic = false;
        this.pivotToCamera = robotToCam;

        dynamicCameras.add(this);
    }

    //dynamic constructor
    /**
     * @param pivotToCamera the camera's fixed mount offset relative to the turret's rotation axis (not the chassis)
     * @param offset the turret's rotation axis, relative to the chassis center
     * @param angularOffset turret angle (relative to the chassis) at a given FPGA timestamp
     */
    public DynamicCamera(String cameraName, Transform3d pivotToCamera, AprilTagFieldLayout tagLayout, BiConsumer<Pose2d, Double> estConsumer, Translation2d offset, Function<Double, Rotation2d> angularOffset) {
        this.photonCamera = new PhotonCamera(cameraName);
        this.poseEstimator = new PhotonPoseEstimator(tagLayout, pivotToCamera);
        this.estConsumer = estConsumer;
        this.isDynamic = true;
        this.pivotToCamera = pivotToCamera;

        this.offset = offset;
        this.angularOffset = angularOffset;

        dynamicCameras.add(this);
    }

    public static void periodic() {
        if (!enabled) return;

        for (final DynamicCamera dynamicCamera : dynamicCameras) {
            for (var result : dynamicCamera.photonCamera.getAllUnreadResults()) {
                for (int i = 0; i < result.targets.size(); i++) {
                    if (result.targets.get(i).poseAmbiguity > dynamicCamera.ambiguityThreshold
                    || result.targets.get(i).bestCameraToTarget.getTranslation().getNorm() < dynamicCamera.minDistThreshold
                    || result.targets.get(i).bestCameraToTarget.getTranslation().getNorm() > dynamicCamera.maxDistThreshold)
                    {
                        result.targets.remove(i);
                        i--;
                    }
                }

                if (result.targets.isEmpty()) continue;

                if (dynamicCamera.isDynamic) {
                    // The camera's true mount transform changes every frame as the turret rotates, so it
                    // has to be recomputed (using the turret angle AT CAPTURE TIME, not "right now") and
                    // pushed into the estimator before asking it to solve this frame's pose.
                    dynamicCamera.poseEstimator.setRobotToCameraTransform(
                        dynamicCamera.computeLiveRobotToCamera(result.getTimestampSeconds())
                    );
                }

                Optional<EstimatedRobotPose> curEst = estimate(dynamicCamera, result);

                if (curEst.isPresent()) {
                    Pose2d estimatedPose = curEst.get().estimatedPose.toPose2d();
                    double timeStamp = curEst.get().timestampSeconds;
                    dynamicCamera.estConsumer.accept(estimatedPose, timeStamp);
                }
            }
        }
    }

    // Decides pose strategy off of this frame's own tag count, not off of loop position.
    private static Optional<EstimatedRobotPose> estimate(DynamicCamera dynamicCamera, PhotonPipelineResult result) {
        if (result.getMultiTagResult().isPresent()) {
            if (!dynamicCamera.isDynamic) {
                // Static mount: the coprocessor's own multi-tag solve (using whatever robotToCamera
                // is configured in the PhotonVision UI) is valid, since that mount never moves.
                return dynamicCamera.poseEstimator.estimateCoprocMultiTagPose(result);
            }

            // Dynamic mount: the coprocessor has no idea what the turret angle is, so its multi-tag
            // solve can never reflect the live transform set above. Solve multi-tag on the RIO instead,
            // which does use the transform we just pushed in.
            var camMatrix = dynamicCamera.photonCamera.getCameraMatrix();
            var distCoeffs = dynamicCamera.photonCamera.getDistCoeffs();
            if (camMatrix.isPresent() && distCoeffs.isPresent()) {
                Optional<EstimatedRobotPose> multiTagEst =
                    dynamicCamera.poseEstimator.estimateRioMultiTagPose(result, camMatrix.get(), distCoeffs.get());
                if (multiTagEst.isPresent()) return multiTagEst;
            }
        }

        return dynamicCamera.poseEstimator.estimateLowestAmbiguityPose(result);
    }

    // Composes chassis -> turret pivot -> (live turret rotation) -> camera into a single transform,
    // using the turret angle the turret actually had when this frame was captured.
    private Transform3d computeLiveRobotToCamera(double timestampSeconds) {
        Rotation2d turretAngle = angularOffset.apply(timestampSeconds);

        Transform3d chassisToPivot = new Transform3d(
            new Translation3d(offset.getX(), offset.getY(), 0), new Rotation3d());
        Transform3d pivotRotation = new Transform3d(
            new Translation3d(), new Rotation3d(0, 0, turretAngle.getRadians()));

        return chassisToPivot.plus(pivotRotation).plus(pivotToCamera);
    }

    public void setThresholds(double newMinDistThreshold, double newMaxDistThreshold, double newAmbiguityThreshold) {
        this.minDistThreshold = newMinDistThreshold;
        this.maxDistThreshold = newMaxDistThreshold;
        this.ambiguityThreshold = newAmbiguityThreshold;
    }

    public static void enable() {
        enabled = true;
    }

    public static void disable() {
        enabled = false;
    }
}
