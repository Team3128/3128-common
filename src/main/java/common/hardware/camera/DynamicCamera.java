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
                dynamicCamera.poseEstimator.setRobotToCameraTransform(dynamicCamera.getRobotToCam());
            }

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
                    dynamicCamera.estConsumer.accept(curEst.get().estimatedPose.toPose2d(), curEst.get().timestampSeconds);
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

    // robot center -> turret pivot, rotated by the turret angle, then pivot -> camera
    private Transform3d getRobotToCam() {
        Transform3d robotToPivot = new Transform3d(
            offset,
            new Rotation3d(0, 0, angularOffset.get().getRadians()));
        return robotToPivot.plus(pivotToCam);
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
