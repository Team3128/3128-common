package common.hardware.camera;

import java.util.LinkedList;
import java.util.Optional;
import java.util.function.BiConsumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class DynamicCamera {

    //static variables
    public static final LinkedList<DynamicCamera> dynamicCameras = new LinkedList<DynamicCamera>();
    private static double minDistThreshold = 0, maxDistThreshold = 100, ambiguityThreshold = 0.2;
    public static boolean enabled = true;

    //dynamic camera related variables
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator poseEstimator;
    private final BiConsumer<Pose2d, Double> estConsumer;
    private final boolean isDynamic;

    // estConsumer sends estimated poses to the SwerveBase where they are factored into the robot's odometry
    public DynamicCamera(String cameraName, Boolean isDynamic, Transform3d robotToCam, AprilTagFieldLayout tagLayout, BiConsumer<Pose2d, Double> estConsumer) {
        this.photonCamera = new PhotonCamera(cameraName);
        this.poseEstimator = new PhotonPoseEstimator(tagLayout, robotToCam);
        this.estConsumer = estConsumer;
        this.isDynamic = isDynamic;

        dynamicCameras.add(this);
    }

    public static void periodic() {
        if (!enabled) return;

        for (final DynamicCamera dynamicCamera : dynamicCameras) {
            Optional<EstimatedRobotPose> curEst = Optional.empty();

            for (var result : dynamicCamera.photonCamera.getAllUnreadResults()) {
                for (int i = 0; i < result.targets.size(); i++) {
                    if (result.targets.get(i).poseAmbiguity > ambiguityThreshold 
                    || result.targets.get(i).bestCameraToTarget.getTranslation().getNorm() < minDistThreshold 
                    || result.targets.get(i).bestCameraToTarget.getTranslation().getNorm() > maxDistThreshold) 
                    {
                        result.targets.remove(i);
                        i--;
                    }                    
                }

                if (curEst.isEmpty()) {
                    curEst = dynamicCamera.poseEstimator.estimateLowestAmbiguityPose(result);
                } else {
                    curEst = dynamicCamera.poseEstimator.estimateCoprocMultiTagPose(result);
                }

                if (curEst.isPresent()) {
                    Pose2d dynamicPose = curEst.get().estimatedPose.toPose2d();
                    double timeStamp = curEst.get().timestampSeconds;

                    if (dynamicCamera.isDynamic) {
                        dynamicPose = new Pose2d();
                        dynamicCamera.estConsumer.accept(dynamicPose, timeStamp);
                    } else {
                        dynamicCamera.estConsumer.accept(dynamicPose, timeStamp);
                    }
                }
            }
        }
    }

    public static void setThresholds(double newMinDistThreshold, double newMaxDistThreshold, double newAmbiguityThreshold) {
        minDistThreshold = newMinDistThreshold;
        ambiguityThreshold = newAmbiguityThreshold;
        maxDistThreshold = newMaxDistThreshold;
    }

    public static void enable() {
        enabled = true;
    }

    public static void disable() {
        enabled = false;
    }
}
