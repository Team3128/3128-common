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

import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
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

    public Pose2d gyroTest = new Pose2d();
    
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
        
    public static void setResources(DoubleSupplier gyro, BiConsumer<Pose2d, Double> odometry, AprilTagFieldLayout aprilTags, Supplier<Pose2d> robotPose) {
        Camera.gyro = gyro;
        Camera.odometry = odometry;
        Camera.aprilTags = aprilTags;
        Camera.robotPose = robotPose;
    }

    public static void setThresholds(double distanceThreshold, double ambiguityThreshold) {
        Camera.distanceThreshold = distanceThreshold;
        Camera.ambiguityThreshold = ambiguityThreshold;
    }

    public void update() {
        if (isDisabled) return;
        hasSeenTag = false;
        result = camera.getLatestResult();

        if (!result.hasTargets()) {
            targets = null;
            Logger.recordOutput("Vision/" + camera.getName(), robotPose.get());
            return;
        }

        final Pose2d estPos = getPose(result);
        gyroTest = getGyroPose(result);

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
        odometry.accept(gyroTest, result.getTimestampSeconds());
    }

    public boolean isGoodEstimate(Pose2d pose){
        return pose.getTranslation().getDistance(robotPose.get().getTranslation()) < validDist;
    }

    public Pose2d getPose(PhotonPipelineResult result) {
        double lowestAmbiguityScore = 10;
        PhotonTrackedTarget lowestAmbiguityTarget = null;

        if (!result.hasTargets()) return new Pose2d();

        for (PhotonTrackedTarget target : result.targets) {
            if (isValidTarget(target) && getPoseAmbiguity(target) < lowestAmbiguityScore && getPoseAmbiguity(target) != -1) {
                lowestAmbiguityScore = getPoseAmbiguity(target);
                lowestAmbiguityTarget = target;
                hasSeenTag = tags.contains(target.getFiducialId());
            }
        }

        if (lowestAmbiguityTarget == null) return new Pose2d();

        Optional<Pose3d> targetPosition = aprilTags.getTagPose(lowestAmbiguityTarget.getFiducialId());
        estimatedPose = targetPosition.get().transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse()).transformBy(offset.inverse()).toPose2d();
        return estimatedPose;
    }

    public Pose2d getFinalPos(PhotonTrackedTarget target) {
        Pose3d test = new Pose3d();
        Pose2d test2 = new Pose2d();
        // test2 = test.transformBy(target.getBestCameraToTarget().inverse()).transformBy(offset.inverse()).getNorm();
        return null;
    }

    public Pose2d getGyroPose(PhotonPipelineResult result) {
        Pose2d pose = getPose(result);
        Rotation2d gyroAngle = Rotation2d.fromDegrees(MathUtil.angleModulus(getGyroAngle()));
        Pose2d updatedPose = new Pose2d(pose.getX(), pose.getY(), gyroAngle); 
        return updatedPose;
    }

    public static void addIgnoredTags(int ...ignoredTags) {
        for(final int tag : ignoredTags) {
            Camera.ignoredTags.add(tag);
        }
    }

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

    public double getGyroAngle() {
        return gyro.getAsDouble();
    }

    public void initShuffleboard() {
        NAR_Shuffleboard.addData(camera.getName(), "Estimated Pose", () -> estimatedPose.toString(), 0, 0, 3, 1);
        NAR_Shuffleboard.addData(camera.getName(), "Is Disabled", () -> isDisabled, 4, 0, 1, 1);
        NAR_Shuffleboard.addData(camera.getName(), "Has target", () -> result.hasTargets(), 4, 0, 1, 1);
        NAR_Shuffleboard.addData(camera.getName(), "gyro", () -> gyroTest.toString(), 5, 0, 1, 1);
    }
}