package common.hardware.camera;
 import java.util.ArrayList;
 import java.util.List;
import java.util.LinkedList;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose3d;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;

public class DynamicCamera {

    //static variables
    public static final LinkedList<DynamicCamera> dynamicCameras = new LinkedList<DynamicCamera>();
    public static boolean enabled = true;
    public boolean withinDist = true;
    public Pose2d estimatedPose = new Pose2d();
    public Pose3d cameraPose = new Pose3d();
     public double distance = 0;
    private PhotonPipelineResult result = new PhotonPipelineResult();
     private List<PhotonPipelineResult> resultList = new ArrayList<PhotonPipelineResult>();
     public boolean isDisabled = false;

    //dynamic camera related variables
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator poseEstimator;
    private final BiConsumer<Pose2d, Double> estConsumer;
    private final boolean isDynamic;
    private Transform3d pivotToCam;
    private Translation3d offset;
    private Supplier<Rotation2d> angularOffset;

    // each turret angle record is matched with the angle from when exactly in time it was captured
    private final TimeInterpolatableBuffer<Rotation2d> angularOffsetHistory = TimeInterpolatableBuffer.createBuffer(1.5);
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

    public DynamicCamera(String cameraName, Transform3d pivotToCam, AprilTagFieldLayout tagLayout, BiConsumer<Pose2d, Double> estConsumer, Translation3d offset, Supplier<Rotation2d> angularOffset) {
        this.photonCamera = new PhotonCamera(cameraName);
        this.poseEstimator = new PhotonPoseEstimator(tagLayout, pivotToCam);
        this.estConsumer = estConsumer;
        this.isDynamic = true;
        this.pivotToCam = pivotToCam;
        
        this.offset = offset;
        this.angularOffset = angularOffset;

        dynamicCameras.add(this);
    }

    public static void periodic() {
        if (!enabled) return;

        for (final DynamicCamera dynamicCamera : dynamicCameras) {
            Optional<EstimatedRobotPose> curEst = Optional.empty();
            
            if (dynamicCamera.isDynamic) {
                dynamicCamera.angularOffsetHistory.addSample(Timer.getFPGATimestamp(), dynamicCamera.angularOffset.get());
            }

            for (var result : dynamicCamera.photonCamera.getAllUnreadResults()) {
                List<PhotonTrackedTarget> cleanList = new ArrayList<>();

                for (int i = 0; i < result.targets.size(); i++) {
                    if (!(result.targets.get(i).poseAmbiguity > dynamicCamera.ambiguityThreshold 
                    || result.targets.get(i).bestCameraToTarget.getTranslation().getNorm() < dynamicCamera.minDistThreshold 
                    || result.targets.get(i).bestCameraToTarget.getTranslation().getNorm() > dynamicCamera.maxDistThreshold))
                    {
                        cleanList.add(result.targets.get(i));
                    }                    
                }

                result.targets = cleanList;

                // prefer the multi-tag estimate, single-tag is only a fallback
                curEst = dynamicCamera.poseEstimator.estimateCoprocMultiTagPose(result);
                if (curEst.isEmpty()) {
                    curEst = dynamicCamera.poseEstimator.estimateLowestAmbiguityPose(result);
                }

                if (curEst.isPresent()) {
                    if (dynamicCamera.isDynamic) {
                        Pose2d modifiedPose = offsetPose(curEst.get().estimatedPose.toPose2d(), curEst.get().timestampSeconds, dynamicCamera);
                        dynamicCamera.estConsumer.accept(modifiedPose, curEst.get().timestampSeconds);
                    } else {
                        dynamicCamera.estConsumer.accept(curEst.get().estimatedPose.toPose2d(), curEst.get().timestampSeconds);
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

    private static Pose2d offsetPose(Pose2d oldPose, double timestamp, DynamicCamera cam) {
        Rotation2d turretAngle = cam.angularOffsetHistory.getSample(timestamp).orElse(cam.angularOffset.get());
        double robotHeading = oldPose.getRotation().getRadians() - turretAngle.getRadians();

        Rotation2d newRotation = new Rotation2d(robotHeading);
        double newX = oldPose.getX() - (cam.offset.getX() * Math.cos(robotHeading) - cam.offset.getY() * Math.sin(robotHeading));
        double newY = oldPose.getY() - (cam.offset.getX() * Math.sin(robotHeading) + cam.offset.getY() * Math.cos(robotHeading));

        Pose2d newPose = new Pose2d(newX, newY, newRotation);

        return newPose;
    }

    public void initShuffleboard() {
        NAR_Shuffleboard.addData(photonCamera.getName(), "WithinDist", () -> withinDist, 2, 2, 1, 1);
        NAR_Shuffleboard.addData(photonCamera.getName(), "Estimated Pose X", () -> estimatedPose.getX(), 0, 0, 2, 1);
        NAR_Shuffleboard.addData(photonCamera.getName(), "Estimated Pose Y", () -> estimatedPose.getY(), 0, 1, 2, 1);
        NAR_Shuffleboard.addData(photonCamera.getName(), "camera pose X", () -> cameraPose.getX(), 2, 0, 2, 1);
        NAR_Shuffleboard.addData(photonCamera.getName(), "camera pose Y", () -> cameraPose.getY(), 2, 1, 2, 1);
        NAR_Shuffleboard.addData(photonCamera.getName(), "dist", () -> distance, 0, 2, 1, 1);
        // NAR_Shuffleboard.addData(photonCamera.getName(), "Has target", () -> result.hasTargets(), 1, 2, 1, 1);
        NAR_Shuffleboard.addData(photonCamera.getName(), "Enabled", () -> !isDisabled, 3, 2);
    }
}
