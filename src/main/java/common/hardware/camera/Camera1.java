package common.hardware.camera;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import common.core.misc.NAR_Robot;
import common.hardware.camera.Camera;
import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Team 3128's class to control the robot's cameras and vision processing.
 * 
 * @since 2024 Crescendo
 * @author Mason Lam, William Yuan
 */
public class Camera1 {

    public PhotonCamera camera;

    private static ArrayList<Integer> targets = new ArrayList<Integer>();

    public static final LinkedList<Camera1> cameras = new LinkedList<Camera1>();

    private PhotonPipelineResult result = new PhotonPipelineResult();

    
    private static DoubleSupplier gyro;
    private static AprilTagFieldLayout aprilTags;
    private static BiConsumer<Pose2d, Double> odometry;
    private static Supplier<Pose2d> robotPose;

    private static double angleThreshold = 30;
    private static double distanceThreshold = 5;
    private static double ambiguityThreshold = 0.5;
    private static boolean multipleTargets = false;

    public static double validDist = 0.5;
    public static double overrideThreshold = 5;
    public static int updateCounter = 0;

    private boolean hasSeenTag;

    public Pose2d estimatedPose = new Pose2d();

    private static ArrayList<Integer> tags = new ArrayList<Integer>();
    private static ArrayList<Integer> ignoredTags = new ArrayList<Integer>();

    public boolean isDisabled = false;

    public double testDist = 0;
    public double testDist2d = 0;
    Translation3d camToTarg = new Translation3d();
    Pose2d test = new Pose2d();
    Pose2d test2 = new Pose2d();

    // private static final HashSet<Integer> reportedErrors = new HashSet<Integer>();

    public Transform3d offset;

    public double gyroAngle;

    public Pose2d gyroTest = new Pose2d();
    
    public Camera1(String name, double xOffset, double yOffset, double yawOffset, double pitchOffset, double rollOffset) {
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
        
    public static void setResources(DoubleSupplier gyro, BiConsumer<Pose2d, Double> odometry, AprilTagFieldLayout aprilTags, Supplier<Pose2d> robotPose) {
        Camera1.gyro = gyro;
        Camera1.odometry = odometry;
        Camera1.aprilTags = aprilTags;
        Camera1.robotPose = robotPose;
    }

    public static void setThresholds(double angleThreshold, double distanceThreshold, double ambiguityThreshold, boolean multipleTargets) {
        Camera1.angleThreshold = angleThreshold;
        Camera1.distanceThreshold = distanceThreshold;
        Camera1.ambiguityThreshold = ambiguityThreshold;
        Camera1.multipleTargets = multipleTargets;
    }

        public void update() {
        if (isDisabled) return;
        hasSeenTag = false;
        result = camera.getLatestResult();

        if (!result.hasTargets()) {
            targets = null;
            return;
        }

        final Pose2d estPos = getGyroPose(result);

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
        double gyroAngle = gyro.getAsDouble() * Math.PI / 180;
        
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
        // public Pose2d getGyroPose(PhotonPipelineResult result) {
    //     Pose2d pose = getPose(result).get();
    //     Rotation2d gyroAngle = Rotation2d.fromDegrees(MathUtil.angleModulus(gyro.getAsDouble()));
    //     Pose2d updatedPose = new Pose2d(pose.getX(), pose.getY(), gyroAngle);        
    //     return updatedPose;
    // }

    public Pose2d getGyroPose(PhotonPipelineResult result) {
        Pose2d pose = getPose(result).get();
        double test = gyro.getAsDouble();
        // Pose2d pose = getPose(result, test).get();
        Rotation2d gyroAngle = Rotation2d.fromDegrees(MathUtil.inputModulus(test, -180, 180));
        Pose2d updatedPose = new Pose2d(pose.getX(), pose.getY(), gyroAngle);
        return updatedPose;
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
            Camera1.ignoredTags.add(tag);
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
        for (final Camera1 camera : cameras) {
            camera.update();
        }
    }

    public static boolean seesTag(){
        for (final Camera1 camera : cameras) {
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
        for (final Camera1 camera : cameras) {
            camera.enable();
        }
    }

    public static void disableAll() {
        for (final Camera1 camera : cameras) {
            camera.disable();
        }
    }

    public void initShuffleboard() {
        // NAR_Shuffleboard.addData(camera.getName(), "Estimated Pose", () -> estimatedPose.toString(), 0, 0, 4, 1);
        // // NAR_Shuffleboard.addData(camera.getName(), "Distance", () -> distance, 0, 1, 2, 1);
        // NAR_Shuffleboard.addData(camera.getName(), "Is Disabled", () -> isDisabled, 2, 1, 1, 1);
        // NAR_Shuffleboard.addData(camera.getName(), "Has target", () -> result.hasTargets(), 3, 2, 1, 1);
    }

    // public void update() {
    //     if (isDisabled) return;
    //     hasSeenTag = false;
    //     result = camera.getLatestResult();

    //     if (!result.hasTargets()) {
    //         targets = null;
    //         Logger.recordOutput("Vision/" + camera.getName(), robotPose.get());
    //         return;
    //     }

    //     // final Pose2d estPos = getPose(result).get();
    //     final Pose2d estPos = getGyroPose(result);
    //     // gyroTest = getGyroPose(result);

    //     if(!isGoodEstimate(estPos)) {
    //         updateCounter++;
    //         if (updateCounter <= overrideThreshold) {
    //             hasSeenTag = false; 
    //             return;
    //         }
    //     }
    //     else {
    //         updateCounter = 0;
    //     }  
    //     // if (!(estPos.getX() == 0 && estPos.getY() == 0)) {
    //     //     odometry.accept(estPos, result.getTimestampSeconds());
    //     // }      
    //     odometry.accept(estPos, result.getTimestampSeconds());
    // }

    // public boolean isGoodEstimate(Pose2d pose){
    //     return pose.getTranslation().getDistance(robotPose.get().getTranslation()) < validDist;
    // }

    // // public Pose2d getPose(PhotonPipelineResult result) {
    // //     double lowestAmbiguityScore = 10;
    // //     PhotonTrackedTarget lowestAmbiguityTarget = null;

    // //     if (!result.hasTargets()) return new Pose2d();

    // //     for (PhotonTrackedTarget target : result.targets) {
    // //         if (isValidTarget(target) && getPoseAmbiguity(target) < lowestAmbiguityScore && getPoseAmbiguity(target) != -1) {
    // //             lowestAmbiguityScore = getPoseAmbiguity(target);
    // //             lowestAmbiguityTarget = target;
    // //             testDist = getDistance(target);
    // //             hasSeenTag = tags.contains(target.getFiducialId()); 
    // //         }
    // //     }

    // //     if (lowestAmbiguityTarget == null) return new Pose2d();

    // //     Optional<Pose3d> targetPosition = aprilTags.getTagPose(lowestAmbiguityTarget.getFiducialId());
    // //     // test = targetPosition.get().transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse()).transformBy(offset.inverse()).toPose2d();
    // //     // estimatedPose = getGyroStablilization(targetPosition, lowestAmbiguityTarget);
    // //     estimatedPose = getGyroStablilization(targetPosition, lowestAmbiguityTarget);
    // //     return estimatedPose;
    // // }

    // public Optional<Pose2d> getPose(PhotonPipelineResult result) {
    //     double lowestAmbiguityScore = 10;
    //     PhotonTrackedTarget lowestAmbiguityTarget = null;

    //     if (!result.hasTargets()) return Optional.empty();

    //     for (PhotonTrackedTarget target : result.targets) {
    //         if (isValidTarget(target) && getPoseAmbiguity(target) < lowestAmbiguityScore && getPoseAmbiguity(target) != -1) {
    //             lowestAmbiguityScore = getPoseAmbiguity(target);
    //             lowestAmbiguityTarget = target;
    //             testDist = getDistance(target);
    //             hasSeenTag = tags.contains(target.getFiducialId()); 
    //         }
    //     }

    //     if (lowestAmbiguityTarget == null) return Optional.empty();

    //     Optional<Pose3d> targetPosition = aprilTags.getTagPose(lowestAmbiguityTarget.getFiducialId());
    //     // test = targetPosition.get().transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse()).transformBy(offset.inverse()).toPose2d();
    //     // estimatedPose = getGyroStablilization(targetPosition, lowestAmbiguityTarget);
    //     estimatedPose = getGyroStablilization(targetPosition, lowestAmbiguityTarget);
    //     return Optional.of(estimatedPose);
    // }

    // public Pose2d getGyroStablilization(Optional<Pose3d> targetPosition, PhotonTrackedTarget bestTarget) {
    //     gyroAngle = gyro.getAsDouble() * Math.PI / 180;
    //     Transform3d targetToCamera = bestTarget.getBestCameraToTarget().inverse();
    //     Rotation3d gyroRotation3d = new Rotation3d(0, 0, gyroAngle);
    //     Pose3d targ = targetPosition.get();
        
    //     Pose3d bruh = new Pose3d(targ.getTranslation().plus(
    //         targetToCamera.getTranslation().rotateBy(targ.getRotation())),
    //         gyroRotation3d.plus(targ.getRotation())
    //     );

    //     Pose3d ok = new Pose3d(bruh.getTranslation().plus(offset.inverse().getTranslation().rotateBy(bruh.getRotation())),
    //         offset.inverse().getRotation().plus(bruh.getRotation())
    //     );
        
    //     return ok.toPose2d();
    // }

    // public Pose2d getGyroPose(PhotonPipelineResult result) {
    //     Pose2d pose = getPose(result).get();
    //     Rotation2d gyroAngle = Rotation2d.fromDegrees(MathUtil.angleModulus(gyro.getAsDouble()));
    //     Pose2d updatedPose = new Pose2d(pose.getX(), pose.getY(), gyroAngle);        
    //     return updatedPose;
    // }

    // public static void addIgnoredTags(int ...ignoredTags) {
    //     for(final int tag : ignoredTags) {
    //         Camera1.ignoredTags.add(tag);
    //     }
    // }

    // public boolean isValidTarget(PhotonTrackedTarget target) {       
    //     return !(getDistance(target) > distanceThreshold) && 
    //         getPoseAmbiguity(target) < ambiguityThreshold &&
    //         !ignoredTags.contains(Integer.valueOf(target.getFiducialId())); // TODO: add angle threshold
    // }

    // public double getDistance(PhotonTrackedTarget target) {
    //     camToTarg = target.getBestCameraToTarget().getTranslation();
    //     testDist2d =  target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
    //     return target.getBestCameraToTarget().getTranslation().getNorm();
    //     // return target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
    // } 

    // public double getPoseAmbiguity(PhotonTrackedTarget target) {
    //     return target.getPoseAmbiguity();
    // } 

    // public static void updateAll() {
    //     for (final Camera1 camera : cameras) {
    //         camera.update();
    //     }
    // }

    // public static boolean seesTag(){
    //     for (final Camera1 camera : cameras) {
    //         if(camera.hasSeenTag) return true;
    //     }
    //     return false;
    // }
    
    // public void enable() {
    //     isDisabled = false;
    // }

    // public void disable(){
    //     isDisabled = true;
    // }

    // public static void enableAll() {
    //     for (final Camera1 camera : cameras) {
    //         camera.enable();
    //     }
    // }

    // public static void disableAll() {
    //     for (final Camera1 camera : cameras) {
    //         camera.disable();
    //     }
    // }

    // public double getGyroAngle() {
    //     return gyro.getAsDouble();
    // }

    // public void initShuffleboard() {
    //     NAR_Shuffleboard.addData(camera.getName(), "Estimated Pose w Gyro Stabilization", () -> estimatedPose.toString(), 0, 0, 3, 1);
    //     NAR_Shuffleboard.addData(camera.getName(), "old method", () -> test2.toString(), 0, 0, 3, 1);
    //     NAR_Shuffleboard.addData(camera.getName(), "Estimated Pose", () -> test.toString(), 0, 1, 3, 1);
    //     NAR_Shuffleboard.addData(camera.getName(), "Is Disabled", () -> isDisabled, 1, 0, 1, 3);
    //     NAR_Shuffleboard.addData(camera.getName(), "Has target", () -> result.hasTargets(), 4, 0, 1, 1);
    //     NAR_Shuffleboard.addData(camera.getName(), "gyro", ()->gyro.getAsDouble(), 0, 3, 1, 1);
    //     // NAR_Shuffleboard.addData(camera.getName(), "gyro", () -> gyroTest.toString(), 5, 0, 1, 1);
    //     // NAR_Shuffleboard.addData(camera.getName(), "distance3d", () -> testDist, 6, 0, 1, 1);
    //     // NAR_Shuffleboard.addData(camera.getName(), "distance2d", () -> testDist2d, 7, 0, 1, 1);
    //     // NAR_Shuffleboard.addData(camera.getName(), "camToTarg", () -> camToTarg.toString(), 8, 0, 1, 1);
    // }
}