package common.hardware.camera;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import common.core.misc.NAR_Robot;
import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
    private static ArrayList<Integer> targets = new ArrayList<Integer>();
    public static final LinkedList<Camera> cameras = new LinkedList<Camera>();
    private PhotonPipelineResult result = new PhotonPipelineResult();  
    
    private static DoubleSupplier gyro;
    private static AprilTagFieldLayout aprilTags;
    private static BiConsumer<Pose2d, Double> odometry;
    private static Supplier<Pose2d> robotPose;
    
    private static double distanceThreshold = 5;
    private static double ambiguityThreshold = 0.5;
    public static double validDist = 0.5;
    public static double overrideThreshold = 5;
    public static int updateCounter = 0;
    
    private boolean hasSeenTag;
    public Pose2d estimatedPose = new Pose2d();
    private static ArrayList<Integer> tags = new ArrayList<Integer>();
    private static ArrayList<Integer> ignoredTags = new ArrayList<Integer>();
    public boolean isDisabled = false;
    public Transform3d offset;
    public double distance;

    /**
     * Creates a Camera object
     */
    public Camera(String name, double xOffset, double yOffset, double yawOffset, double pitchOffset, double rollOffset) {
        Log.info("Camera", "camera constructer");

        this.offset = new Transform3d(xOffset, yOffset, 0, 
            new Rotation3d(0.0, pitchOffset, 0.0)
            .rotateBy(new Rotation3d(rollOffset, 0, 0))
            .rotateBy(new Rotation3d(0.0,0.0,yawOffset))
        );

        camera = new PhotonCamera(name);

        cameras.add(this);

        initShuffleboard();
        hasSeenTag = false;
    }

    /**
     * Sets the necessary resources for cameras to function.
     * 
     * @param gyro Feeds the angle of the robot.
     * @param odometry Feeds the robot odometry object for vision estimates to update.
     * @param aprilTags Sets the AprilTag positions on the field.
     * @param robotPose Supplies the robot's current pose.
    */
    public static void setResources(DoubleSupplier gyro, BiConsumer<Pose2d, Double> odometry, AprilTagFieldLayout aprilTags, Supplier<Pose2d> robotPose) {
        Camera.gyro = gyro;
        Camera.odometry = odometry;
        Camera.aprilTags = aprilTags;
        Camera.robotPose = robotPose;
    }

    /**
     * Sets the necessary thresholds for the cameras to use
     * @param distanceThreshold Maximum distance from tag to accept
     * @param ambiguityThreshold Maximum tag ambiguity to accept
     */
    public static void setThresholds(double distanceThreshold, double ambiguityThreshold) {
        Camera.distanceThreshold = distanceThreshold;
        Camera.ambiguityThreshold = ambiguityThreshold;
    }

    /**
     * Gets the latest camera updates
     */
    public void update() {
        if (isDisabled) return;
        hasSeenTag = false;
        result = camera.getLatestResult();

        if (!result.hasTargets()) {
            targets = null;
            if (NAR_Robot.logWithAdvantageKit) Logger.recordOutput("Vision/" + camera.getName(), robotPose.get());
            return;
        }

        final Pose2d estPos = getPose(result).get();

        /*
         * Checks if the the robot has a good estimate
         */

        if(!isGoodEstimate(estPos)) {
            updateCounter++;
            if (updateCounter <= overrideThreshold) {
                hasSeenTag = false; 
                return;
            }
        }
        else {
            updateCounter = 0;
        } 
        
        odometry.accept(estPos, result.getTimestampSeconds());
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

        distance = getDistance(lowestAmbiguityTarget);

        /*
         * Finds the pose of the lowest ambiguity target and uses the gyro to determine the estimated pose
         */
        Optional<Pose3d> targetPosition = aprilTags.getTagPose(lowestAmbiguityTarget.getFiducialId());
        estimatedPose = getGyroStablilization(targetPosition, lowestAmbiguityTarget);

        return Optional.of(estimatedPose);
    }

    /**
     * Uses the gyro angle and target position to determine robot pose
     * @param targetPosition The Pose3d of the target
     * @param bestTarget Lowest ambiguity target
     * @return The estimated Pose2d of the robot
     */
    public Pose2d getGyroStablilization(Optional<Pose3d> targetPosition, PhotonTrackedTarget bestTarget) {
        double gyroAngle = getGyroAngle() * Math.PI / 180;
        
        Transform3d targetToCamera = bestTarget.getBestCameraToTarget().inverse();
        Rotation3d gyroRotation3d = new Rotation3d(0, 0, gyroAngle);
        Pose3d target = targetPosition.get();
        
        /*
         * Determines the position of the camera using the position of the target and the gyro rotation
         */
        Pose3d cameraPose = new Pose3d(target.getTranslation().plus(
            targetToCamera.getTranslation().rotateBy(target.getRotation())),
            gyroRotation3d.plus(target.getRotation())
        );

        /*
         * Determines the position of the robot using the position of the camera and its offset
         */
        Pose3d robotPose = new Pose3d(cameraPose.getTranslation().plus(
            offset.inverse().getTranslation().rotateBy(cameraPose.getRotation())),
            offset.inverse().getRotation().plus(cameraPose.getRotation())
        );
        
        return robotPose.toPose2d();
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
        return !(getDistance(target) > distanceThreshold) && 
            getPoseAmbiguity(target) < ambiguityThreshold &&
            !ignoredTags.contains(Integer.valueOf(target.getFiducialId()));
    }

    public double getDistance(PhotonTrackedTarget target) {
        return target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
    }

    public double getPoseAmbiguity(PhotonTrackedTarget target) {
        return target.getPoseAmbiguity();
    } 

    public double getGyroAngle() {
        return gyro.getAsDouble();
    }

    public static void updateAll() {
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
        NAR_Shuffleboard.addData(camera.getName(), "Estimated Pose", () -> estimatedPose.toString(), 0, 0, 4, 1);
        NAR_Shuffleboard.addData(camera.getName(), "Distance", () -> distance, 0, 1, 2, 1);
        NAR_Shuffleboard.addData(camera.getName(), "Is Disabled", () -> isDisabled, 2, 1, 1, 1);
        NAR_Shuffleboard.addData(camera.getName(), "Has target", () -> result.hasTargets(), 3, 2, 1, 1);
    }
}