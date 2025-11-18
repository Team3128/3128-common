package common.hardware.camera;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import common.hardware.camera.Camera;
import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Team 3128's class to control the robot's cameras and vision processing.
 * 
 * @since 2024 Crescendo
 * @author Audrey Zheng
 */

public class Camera {

    public PhotonCamera camera;

    public static final LinkedList<Camera> cameras = new LinkedList<Camera>();

    private PhotonPipelineResult result = new PhotonPipelineResult();
    private List<PhotonPipelineResult> resultList = new ArrayList<PhotonPipelineResult>();
    
    private static DoubleSupplier gyro;
    private static AprilTagFieldLayout aprilTags;
    private static BiConsumer<Pose2d, Double> odometry;
    private static Supplier<Pose2d> robotPose;

    private double maxDistanceThreshold = 5;
    private double ambiguityThreshold = 0.5;
    private double minDistanceThreshold = 0.34;

    public static double validDist = 0.5; //meters
    public static double overrideThreshold = 5;
    public static int updateCounter = 0;

    private boolean hasSeenTag;

    public Pose2d estimatedPose = new Pose2d();

    private static ArrayList<Integer> tags = new ArrayList<Integer>();
    private static ArrayList<Integer> ignoredTags = new ArrayList<Integer>();

    public boolean isDisabled = false;

    public Transform3d offset;

    public PhotonPoseEstimator poseEstimator;

    public double gyroAngle;

    public Pose2d gyroTest = new Pose2d();

    public double distance = 0;

    public boolean withinDist = true;

    public Pose3d cameraPose = new Pose3d();
    
    public Camera(String name, double xOffset, double yOffset, double yawOffset, double pitchOffset, double rollOffset) {
        Log.info("Camera", "camera constructer");

        this.offset = new Transform3d(xOffset, yOffset, 0, 
            new Rotation3d(0.0, pitchOffset, 0.0)
            .rotateBy(new Rotation3d(rollOffset, 0, 0))
            .rotateBy(new Rotation3d(0.0,0.0,yawOffset))
        );

        this.poseEstimator = new PhotonPoseEstimator(aprilTags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, offset);

        camera = new PhotonCamera(name);

        cameras.add(this);

        // initShuffleboard();
        hasSeenTag = false;
    }
        
    public static void setResources(DoubleSupplier gyro, BiConsumer<Pose2d, Double> odometry, Supplier<Pose2d> robotPose, AprilTagFieldLayout aprilTags) {
        Camera.gyro = gyro;
        Camera.odometry = odometry;
        Camera.robotPose = robotPose;
        Camera.aprilTags = aprilTags;
    }

    public void setThresholds(double minDistanceThreshold, double maxDistanceThreshold, double ambiguityThreshold) {
        this.minDistanceThreshold = minDistanceThreshold;
        this.maxDistanceThreshold = maxDistanceThreshold;
        this.ambiguityThreshold = ambiguityThreshold;
    }

    public double lowestAmbiguityInResult(PhotonPipelineResult result) {
        double lowestAmbiguityScore = 10;

        if (!result.hasTargets()) return 100000;

        for (PhotonTrackedTarget target : result.targets) {
            if (isValidTarget(target) && getPoseAmbiguity(target) < lowestAmbiguityScore && getPoseAmbiguity(target) != -1) {
                lowestAmbiguityScore = getPoseAmbiguity(target);
            }
        }

        return lowestAmbiguityScore;
    }

    public double averageAmbiguityInResult(PhotonPipelineResult result) {
        double totalAmbiguity = 0;
        double numberOfTargets = 0;

        if (!result.hasTargets()) return 100000;

        for (PhotonTrackedTarget target : result.targets) {
            if (isValidTarget(target) && getPoseAmbiguity(target) != -1) {
                if (getPoseAmbiguity(target) > ambiguityThreshold) {
                    totalAmbiguity += getPoseAmbiguity(target);
                    numberOfTargets += 1;
                }
            }
        }

        return totalAmbiguity/numberOfTargets;
    }

    public void update() {
        if (isDisabled) return;
        hasSeenTag = false;
        resultList = camera.getAllUnreadResults();

        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        
        for (PhotonPipelineResult result : resultList) {
            visionEst = poseEstimator.update(result);
        
            visionEst.ifPresent(
                Pose -> {
                Pose2d estPos = Pose.estimatedPose.toPose2d();
                if(!isGoodEstimate(estPos)) { //checks if the estimated pose is within a valid distance from the robot pose :)
                    odometry.accept(estPos, result.getTimestampSeconds());
                } 
            });
        }

        //old stuff
        // for (PhotonPipelineResult result : resultList) {
        //     if (!result.hasTargets()) {
                
        //     }
            
        //     // Optional<Pose2d> estPosOpt = getGyroPose(result);
        //     // if (estPosOpt.isEmpty()) return cameraResult;
        //     // Pose2d estPos = estPosOpt.get();

            
    
        //     // if (estPos.minus(new Pose2d(0,0,Rotation2d.fromDegrees(0))).getTranslation().getNorm() <= 0.05)
        //     //     return cameraResult;
        // }
    }


    /**
     * Gets the latest camera updates
     * @param result Latest result from the camera
     * @return The estimated robot pose
     */
    public Optional<Pose2d> getPose(PhotonPipelineResult result) {
        double lowestAmbiguityScore = 10;
        PhotonTrackedTarget lowestAmbiguityTarget = null;

        if (!result.hasTargets()) return Optional.empty();

        /*
         * Find the target with the lowest ambiguity score 
         */
        for (PhotonTrackedTarget target : result.targets) {
            if (isValidTarget(target) && getPoseAmbiguity(target) < lowestAmbiguityScore && getPoseAmbiguity(target) != -1) {
                lowestAmbiguityScore = getPoseAmbiguity(target);
                lowestAmbiguityTarget = target;
                hasSeenTag = tags.contains(target.getFiducialId()); 
            }
        }

        if (lowestAmbiguityTarget == null) return Optional.empty();
        /*
         * Finds the pose of the lowest ambiguity target and uses the gyro to determine the estimated pose
         */
        Optional<Pose3d> targetPosition = aprilTags.getTagPose(lowestAmbiguityTarget.getFiducialId());
        
        if (targetPosition.isEmpty()) return Optional.empty();

        cameraPose = targetPosition.get().transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse());
        estimatedPose = cameraPose.transformBy(offset.inverse()).toPose2d();

        return Optional.of(estimatedPose);
    }

    public Optional<Pose2d> getGyroPose(PhotonPipelineResult result) {
        Optional<Pose2d> poseOpt = getPose(result);
        if (poseOpt.isEmpty()) return Optional.empty();
        Pose2d pose = poseOpt.get();
        double gyroUnconstrained = gyro.getAsDouble();

        Rotation2d gyroAngle = Rotation2d.fromDegrees(MathUtil.inputModulus(gyroUnconstrained, -180, 180));
        Pose2d updatedPose = new Pose2d(pose.getX(), pose.getY(), gyroAngle);
        return Optional.of(updatedPose);
    }

    /**
     * Returns if an estimate is within a valid distance from the robot pose
     * @param pose Estimated pose
     * @return If the estimated pose is valid
     */
    public boolean isGoodEstimate(Pose2d pose){
        return pose.getTranslation().getDistance(robotPose.get().getTranslation()) < validDist;
    }

    /**
     * Blacklists tags from use
     * @param ignoredTags IDs of ignored tags
     */
    public static void addIgnoredTags(int ...ignoredTags) {
        for(final int tag : ignoredTags) {
            Camera.ignoredTags.add(tag);
        }
    }

    /**
     * Returns true if the target is within the distance and ambiguity thresholds and is not blacklisted
     * @param target 
     * @return If the target is valid
     */
    public boolean isValidTarget(PhotonTrackedTarget target) {       
        return getDistance(target) < maxDistanceThreshold &&
            getDistance(target) > minDistanceThreshold &&
            getPoseAmbiguity(target) < ambiguityThreshold &&
            !ignoredTags.contains(Integer.valueOf(target.getFiducialId()));
    }

    public double getDistance(PhotonTrackedTarget target) {
        distance = target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
        return target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
    }

    public double getPoseAmbiguity(PhotonTrackedTarget target) {
        return target.getPoseAmbiguity();
    } 

    public double getGyroAngle() {
        return gyro.getAsDouble();
    }

    public static void updateAll() {
        //all cameras are updated no matter what :)
        for (final Camera camera : cameras) {
            camera.update();
        }
    }

    public static boolean seesTag(){
        for (final Camera camera : cameras) {
            if(camera.hasSeenTag) return true;
        }
        return false;
    }
    
    public void enable() {
        isDisabled = false;
    }

    public void disable(){
        isDisabled = true;
    }

    public static void enableAll() {
        for (final Camera camera : cameras) {
            camera.enable();
        }
    }

    public static void disableAll() {
        for (final Camera camera : cameras) {
            camera.disable();
        }
    }

    public void initShuffleboard() {
        NAR_Shuffleboard.addData(camera.getName(), "WithinDist", () -> withinDist, 0, 2, 3, 1);
        NAR_Shuffleboard.addData(camera.getName(), "Estimated Pose", () -> estimatedPose.toString(), 0, 0, 3, 1);
        NAR_Shuffleboard.addData(camera.getName(), "Has target", () -> result.hasTargets(), 1, 2, 1, 1);
        NAR_Shuffleboard.addData(camera.getName(), "camera pose", () -> cameraPose.toString(), 0, 1, 3, 1);
        NAR_Shuffleboard.addData(camera.getName(), "dist", () -> distance, 2, 2, 1, 1);

    }
}