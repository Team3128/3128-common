package common.hardware.camera;

import java.util.LinkedList;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import common.core.misc.NAR_Robot;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.HashSet;
import edu.wpi.first.apriltag.AprilTagFields;

/**
 * Team 3128's class to control the robot's cameras and vision processing.
 * 
 * @since 2022 Rapid React
 * @author Mason Lam, William Yuan 
 */
public class Camera {

    private static ArrayList<Integer> tags = new ArrayList<Integer>();

    private boolean hasSeenTag;

    public static int updateCounter = 0;

    public static double validDist = 0.5;
    public static double overrideThreshold = 5;

    private final PhotonCamera camera;
    private final Transform3d offset;

    private boolean isDisabled = false;
    private PhotonPipelineResult lastResult;
    private EstimatedRobotPose lastPose;

    private static AprilTagFieldLayout aprilTags;
    // private static HashMap<Integer, Pose3d> aprilTags;
    private static PoseStrategy calc_strategy;
    private static BiConsumer<Pose2d, Double> odometry;
    private static Supplier<Pose2d> robotPose;
    private static double ambiguityThreshold = 0.3;

    private static ArrayList<Integer> ignoredTags = new ArrayList<Integer>();

    public static final LinkedList<Camera> cameras = new LinkedList<Camera>();

    private static final HashSet<Integer> reportedErrors = new HashSet<Integer>();

    private double distanceThreshold = 3.5;
    
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

        cameras.add(this);
        hasSeenTag = false;
    }
    public double getDistanceGround(){
        // double yOffset =this.offset.getY();
        // double pitchOffset = this.offset.getRotation().getY();
        // double pitchObject = Math.toRadians(target.getPitch());
        PhotonTrackedTarget target = lastResult.getBestTarget();
        double objectDistance = target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
        return objectDistance;


    }

    public static void configCameras(AprilTagFields aprilTagLayout, PoseStrategy calc_strategy, BiConsumer<Pose2d, Double> odometry, Supplier<Pose2d> robotPose){
        Camera.aprilTags = aprilTagLayout.loadAprilTagLayoutField();
        Camera.calc_strategy = calc_strategy;
        Camera.odometry = odometry;
        Camera.robotPose = robotPose;
    }

    public static void addIgnoredTags(int ...ignoredTags) {
        for(final int tag : ignoredTags) {
            Camera.ignoredTags.add(tag);
        }
    }

    public static void addTags(int ...tags) {
        for(final int tag : tags) {
            Camera.tags.add(tag);
        }
    }

    public static boolean seesTag(){
        for (final Camera camera : cameras) {
            if(camera.hasSeenTag) return true;
        }
        return false;
    }

    public static void updateAll(){
        for (final Camera camera : cameras) {
            camera.update();
        }
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

    public static void setAmbiguityThreshold(double ambiguityThreshold) {
        Camera.ambiguityThreshold = ambiguityThreshold;
    }

    public void setCamDistanceThreshold(double distThreshold) {
        distanceThreshold = distThreshold;
    }

    public static void setAllDistanceThreshold(double distThreshold) {
        for (final Camera camera : cameras) {
            camera.setCamDistanceThreshold(distThreshold);
        }
    }

    private Optional<EstimatedRobotPose> getEstimatedPose(PhotonPipelineResult result) {
        PhotonTrackedTarget lowestAmbiguityTarget = null;

        double lowestAmbiguityScore = 10;

        for (PhotonTrackedTarget target : result.targets) {
            double targetPoseAmbiguity = target.getPoseAmbiguity();
            double dist = target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
            if (dist > distanceThreshold || targetPoseAmbiguity > ambiguityThreshold || ignoredTags.contains(Integer.valueOf(target.getFiducialId()))) continue;
            // Make sure the target is a Fiducial target.

            if (targetPoseAmbiguity != -1 && targetPoseAmbiguity < lowestAmbiguityScore) {
                lowestAmbiguityScore = targetPoseAmbiguity;
                lowestAmbiguityTarget = target;
                hasSeenTag = tags.contains(target.getFiducialId());
            }
        }

        // Although there are confirmed to be targets, none of them may be fiducial
        // targets.
        if (lowestAmbiguityTarget == null) return Optional.empty();

        int targetFiducialId = lowestAmbiguityTarget.getFiducialId();

        Optional<Pose3d> targetPosition = aprilTags.getTagPose(targetFiducialId);
        // Optional<Pose3d> targetPosition = Optional.of(aprilTags.get(targetFiducialId));

        if (targetPosition.isEmpty()) {
            if (NAR_Robot.logWithAdvantageKit) Logger.recordOutput("Vision/" + camera.getName() + "/Target",  robotPose.get());
            reportFiducialPoseError(targetFiducialId);
            return Optional.empty();
        }

        if (NAR_Robot.logWithAdvantageKit) Logger.recordOutput("Vision/" + camera.getName() + "/Target",  targetPosition.get().toPose2d());

        return Optional.of(
                new EstimatedRobotPose(
                        targetPosition
                                .get()
                                .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                                .transformBy(offset.inverse()),
                        result.getTimestampSeconds(),
                        result.getTargets(),
                        PoseStrategy.LOWEST_AMBIGUITY));
    }

    private void reportFiducialPoseError(int fiducialId) {
        if (!reportedErrors.contains(fiducialId)) {
            DriverStation.reportError(
                    "[PhotonPoseEstimator] Tried to get pose of unknown AprilTag: " + fiducialId, false);
            reportedErrors.add(fiducialId);
        }
    }

    public void update(){
        if (isDisabled) return;
        hasSeenTag = false;
        lastResult = camera.getLatestResult();
        if (!lastResult.hasTargets() && NAR_Robot.logWithAdvantageKit) {
            Logger.recordOutput("Vision/" + camera.getName() + "/Position", robotPose.get());
            return;
        }

        final Optional<EstimatedRobotPose> estimatedPose = getEstimatedPose(lastResult);

        if(!estimatedPose.isPresent() && NAR_Robot.logWithAdvantageKit) {
            Logger.recordOutput("Vision/" + camera.getName() + "/Position", robotPose.get());
        }

        lastPose = estimatedPose.get();
        Pose2d estPose = lastPose.estimatedPose.toPose2d();

        if(!isGoodEstimate(estPose)) {
            updateCounter++;
            if (updateCounter <= overrideThreshold) {
                hasSeenTag = false;
                return;
            }
        }
        else {
            updateCounter = 0;
        }

        if (NAR_Robot.logWithAdvantageKit) Logger.recordOutput("Vision/" + camera.getName() + "/Position", estPose);
        
        odometry.accept(estPose, lastPose.timestampSeconds);
    }

    public boolean isGoodEstimate(Pose2d pose){
        return pose.getTranslation().getDistance(robotPose.get().getTranslation()) < validDist;
    }

    public void initShuffleboard() {
        NAR_Shuffleboard.addData(camera.getName(), "Estimated Pose", ()-> lastPose.estimatedPose.toPose2d().toString(), 0, 0, 3, 1);
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