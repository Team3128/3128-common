package common.hardware.camera;

import java.util.LinkedList;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public class DynamicCamera {

    //static variables
    public static final LinkedList<DynamicCamera> dynamicCameras = new LinkedList<DynamicCamera>();
    public static boolean enabled = true;

    //dynamic camera related variables
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator poseEstimator;
    private final BiConsumer<Pose2d, Double> estConsumer;
    private final boolean isDynamic;
    private double xOffset;
    private double yOffset;
    private Supplier<Rotation2d> angularOffset;
    private Supplier<Pose2d> robotPose2d;
    private double minDistThreshold = 0, maxDistThreshold = 100, ambiguityThreshold = 0.2;

    // estConsumer sends estimated poses to the SwerveBase where they are factored into the robot's odometry
    //not dynamic constructor
    public DynamicCamera(String cameraName, Transform3d robotToCam, AprilTagFieldLayout tagLayout, BiConsumer<Pose2d, Double> estConsumer) {
        this.photonCamera = new PhotonCamera(cameraName);
        this.poseEstimator = new PhotonPoseEstimator(tagLayout, robotToCam);
        this.estConsumer = estConsumer;
        this.isDynamic = false;

        dynamicCameras.add(this);
    }

    //dynamic constructor
    public DynamicCamera(String cameraName, Transform3d robotToCam, AprilTagFieldLayout tagLayout, BiConsumer<Pose2d, Double> estConsumer, double xOffset, double yOffset, Supplier<Rotation2d> angularOffset, Supplier<Pose2d> robotPose2d) {
        this.photonCamera = new PhotonCamera(cameraName);
        this.poseEstimator = new PhotonPoseEstimator(tagLayout, robotToCam);
        this.estConsumer = estConsumer;
        this.isDynamic = true;
        
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.angularOffset = angularOffset;

        this.robotPose2d = robotPose2d;

        dynamicCameras.add(this);
    }

    public static void periodic() {
        if (!enabled) return;

        for (final DynamicCamera dynamicCamera : dynamicCameras) {
            Optional<EstimatedRobotPose> curEst = Optional.empty();

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

                if (curEst.isEmpty()) {
                    curEst = dynamicCamera.poseEstimator.estimateLowestAmbiguityPose(result);
                } else {
                    curEst = dynamicCamera.poseEstimator.estimateCoprocMultiTagPose(result);
                }

                if (curEst.isPresent()) {
                    Pose2d dynamicPose = curEst.get().estimatedPose.toPose2d();
                    double timeStamp = curEst.get().timestampSeconds;
                    
                    if (dynamicCamera.isDynamic) {
                        //the dynamic pose you had was center of the turret
                        Pose2d currentRobotPose2d = dynamicCamera.robotPose2d.get();

                        Rotation2d newRotation = dynamicPose.getRotation().plus(dynamicCamera.angularOffset.get());
                        double newX = dynamicPose.getX() - dynamicCamera.xOffset * newRotation.getCos();
                        double newY = dynamicPose.getY() - dynamicCamera.yOffset * newRotation.getSin();

                        //the line below modifies the result to make it centered at teh robot
                        dynamicPose = new Pose2d(newX, newY, newRotation);
                        dynamicCamera.estConsumer.accept(dynamicPose, timeStamp);
                    } else {
                        dynamicCamera.estConsumer.accept(dynamicPose, timeStamp);
                    }
                }
            }
        }
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
