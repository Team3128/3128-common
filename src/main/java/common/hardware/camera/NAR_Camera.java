package common.hardware.camera;

import java.util.LinkedList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Team 3128's streamlined {@link PhotonCamera} class that provides additional functionality and ease of use.
 * <p> Geometry: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/index.html.
 * <p> Photonvision: https://docs.photonvision.org/en/latest/.
 * 
 * @since 2023 Charged Up
 * @author Mason Lam, William Yuan, Lucas Han, Audrey Zheng
 */
public class NAR_Camera extends PhotonCamera {

    public final Camera camera;

    private PhotonPipelineResult result;

    private List<PhotonTrackedTarget> targets;
    private PhotonTrackedTarget bestTarget;

    private static DoubleSupplier gyro;
    private static BiConsumer<Pose2d, Double> odometry;
    private static HashMap<Integer, Pose2d> aprilTags;

    private static double angleThreshold = 30;
    private static double distanceThreshold = 5;
    private static double ambiguityThreshold = 0.5;
    private static boolean multipleTargets = false;

    private final double FIELD_X_LENGTH = Units.inchesToMeters(648);
    private final double FIELD_Y_LENGTH = Units.inchesToMeters(324);

    /**
     * Creates a NAR_Camera object.
     * 
     * @param camera Specs of the camera.
     */
    public NAR_Camera(Camera camera) {
        super(camera.name);
        this.camera = camera;
        setVersionCheckEnabled(false);
    }

    /**
     * Sets the necessary resources for cameras to function.
     * 
     * @param gyro Feeds the angle of the robot.
     * @param odometry Feeds the robot odometry object for vision estimates to update.
     * @param aprilTags Sets the AprilTag positions on the field.
     */
    public static void setResources(DoubleSupplier gyro, BiConsumer<Pose2d, Double> odometry, HashMap<Integer, Pose2d> aprilTags) {
        NAR_Camera.gyro = gyro;
        NAR_Camera.odometry = odometry;
        NAR_Camera.aprilTags = aprilTags;
    }

    /**
     * Sets the thresholds for camera pose estimates.
     * <p>Defaults: 30 degrees, 5 meters, 0.5, false.
     * @param angleThreshold Angle offset to stop using estimates.
     * @param distanceThreshold Distance to stop accepting updates.
     * @param ambiguityThreshold Pose ambiguity from 0.0 to 1.0 to reject estimates.
     * @param multipleTargets Update position using multiple tags.
     */
    public static void setThresholds(double angleThreshold, double distanceThreshold, double ambiguityThreshold, boolean multipleTargets) {
        NAR_Camera.angleThreshold = angleThreshold;
        NAR_Camera.distanceThreshold = distanceThreshold;
        NAR_Camera.ambiguityThreshold = ambiguityThreshold;
        NAR_Camera.multipleTargets = multipleTargets;
    }

    /**
     * Allows the camera to update robot odometry.
     */
    public void enable() {
        camera.enabled = true;
    }

    /**
     * Stops the camera from updating robot odometry.
     */
    public void disable() {
        camera.enabled = false;
    }

    /**
     * Gets the latest targets.
     */
    public void update() {
        //Don't update if camera is disabled
        if (!camera.enabled) return;

        //returns the most recent camera frame
        result = this.getLatestResult();

        // if camera sees no target, set values to null and return
        if (!result.hasTargets()) {
            targets = null;
            bestTarget = null;
            return;
        }

        targets = result.getTargets();
        bestTarget = result.getBestTarget();

        updatePose();
    }

    /**
     * Sends the camera's pose estimate.
     */
    private void updatePose() {
        final LinkedList<Pose2d> possiblePoses = new LinkedList<Pose2d>();

        // add valid targets to possiblePoses
        for (final PhotonTrackedTarget curTarget : targets) {
            final Pose2d estimate = getPos(curTarget);

            // if target is tolerable, add to possiblePoses
            if (!estimate.equals(new Pose2d()) && isValidTarget(curTarget)) {

                possiblePoses.add(estimate);
            }

            // if camera has multiple targets disabled, break after adding first valid target
            if (!multipleTargets) break;
        }

        // updates robot with all acceptable poses from possiblePoses
        for (final Pose2d curPos : possiblePoses) {
            if (translationOutOfBounds(curPos.getTranslation())) return;
            odometry.accept(curPos, result.getTimestampSeconds());
        }
    }

    /**
     * Returns whether or not to accept the pose estimate.
     * @param target An AprilTag.
     * @return If the AprilTag is within set constraints.
     */
    private boolean isValidTarget(PhotonTrackedTarget target) {
        final Rotation2d relativeAngle = getAccTarget(target).getRotation();
        return targetAmbiguity(target) < ambiguityThreshold
            && !(getDistance(target) > distanceThreshold || Math.abs(relativeAngle.getDegrees()) < 180 - angleThreshold);
    }

    /**
     * Returns whether a translation is on the field.
     * @param translation A calculated translation.
     * @return If the Translation2d is within the bounds of the field.
     */
    private boolean translationOutOfBounds(Translation2d translation) {
        return translation.getX() > FIELD_X_LENGTH || translation.getX() < 0 || translation.getY() > FIELD_Y_LENGTH
                || translation.getY() < 0;
    }

    /**
     * Returns whether or not the camera sees a target.
     * @return If camera sees any targets.
     */
    public boolean hasTarget() {
        return targets != null;
    }

    /**
     * Returns the distance from the best target to the camera.
     * @return The distance in meters.
     */
    public double getDistance() {
        return getDistance(bestTarget);
    }

    /**
     * Returns the distance from an AprilTag to the camera.
     * @param target An AprilTag.
     * @return The distance in meters.
     */
    private double getDistance(PhotonTrackedTarget target) {
        if (!hasTarget()) return -1;

        final Transform2d transform = getRelTarget(target);
        return Math.hypot(transform.getX(), transform.getY());
    }

    /**
     * Returns the ID of the best target.
     * @return The target ID as an integer.
     */
    public int targetId() {
        return targetId(bestTarget);
    }

    /**
     * Returns the ID of an AprilTag.
     * @param target An AprilTag
     * @return The target ID as an integer.
     */
    private int targetId(PhotonTrackedTarget target) {
        return hasTarget() ? target.getFiducialId() : 0;
    }
    /**
     * Returns the ambiguity of the best target with lower being more accurate.
     * @return A value from 0.0 to 1.0 representing the accuracy of the target.
     */
    public double targetAmbiguity() {
        return targetAmbiguity(bestTarget);
    }

    /**
     * Returns the ambiguity of an AprilTag with lower being more accurate.
     * @param target An AprilTag.
     * @return A value from 0.0 to 1.0 representing the accuracy of the target.
     */
    private double targetAmbiguity(PhotonTrackedTarget target) {
        return hasTarget() ? target.getPoseAmbiguity() : 0;
    }

    /**
     * Returns the best target to camera vector as a Transform3d.
     * @return Transform3d with coordinate system relative to camera.
     */
    public Transform3d getTarget3d() {
        return getTarget3d(bestTarget);
    }

    /**
     * Returns an AprilTag to camera vector as a Transform3d.
     * @param target An AprilTag.
     * @return Transform3d with coordinate system relative to camera.
     */
    private Transform3d getTarget3d(PhotonTrackedTarget target) {
        return hasTarget() ? target.getBestCameraToTarget() : new Transform3d();
    }

    /**
     * Returns the best target to camera vector as a Transform2d.
     * @return Transform2d with coordinate system relative to camera.
     */
    public Transform2d getRelTarget() {
        return getRelTarget(bestTarget);
    }

    /**
     * Returns an AprilTag to camera vector as a Transform2d.
     * @param target An AprilTag.
     * @return Transform2d with coordinate system relative to camera.
     */
    private Transform2d getRelTarget(PhotonTrackedTarget target) {
        if (!hasTarget()) return new Transform2d();

        final Transform3d transform = getTarget3d(target);

        final Translation2d translation2d = transform.getTranslation().toTranslation2d();
        final Rotation2d rotation2d = transform.getRotation().toRotation2d().unaryMinus();

        return new Transform2d(translation2d, rotation2d);
    }

    /**
     * Returns the best target to camera vector as a Transform2d.
     * @return Transform2d with coordinate system relative to target.
     */
    public Transform2d getAccTarget() {
        return getAccTarget(bestTarget);
    }

    /**
     * Returns an AprilTag to camera vector as a Transform2d.
     * @param target An AprilTag.
     * @return Transform2d with coordinate system relative to target.
     */
    private Transform2d getAccTarget(PhotonTrackedTarget target) {
        // if no valid target, return empty Transform2d
        if (!hasTarget() || !aprilTags.containsKey(targetId(target))) return new Transform2d();

        // angle of the AprilTag relative to the camera
        final Rotation2d relTargetAngle;

        // use gyro if resource is supplied
        if (gyro == null) {
            relTargetAngle = getRelTarget(target).getRotation();
        }
        else {
            // angle of the AprilTag relative to the field
            final double fieldTargetAngle = aprilTags.get(targetId(target)).getRotation().getDegrees();

            // angle of the robot relative to the AprilTag using gyro measurement
            final double robotAngle = MathUtil.inputModulus(gyro.getAsDouble() + fieldTargetAngle + camera.offset.getRotation().getDegrees() + 180,-180,180);

            relTargetAngle = Rotation2d.fromDegrees(robotAngle);
        }

        // vector relative to camera coordinate system
        final Translation2d vector = getRelTarget(target).getTranslation().rotateBy(relTargetAngle);

        return new Transform2d(vector, relTargetAngle);
    }

    /**
     * Returns position of the robot on the field based on the best target.
     * @return Pose estimate.
     */
    public Pose2d getPos() {
        return getPos(bestTarget);
    }

    /**
     * Returns position of the robot on the field based on an AprilTag.
     * @param target An AprilTag.
     * @return Pose estimate.
     */
    private Pose2d getPos(PhotonTrackedTarget target) {
        final Pose2d AprilTag = aprilTags.get(targetId(target));

        // if no valid target, return empty Pose2d
        if (!hasTarget() || !aprilTags.containsKey(targetId(target)) || target == null) return new Pose2d();

        // vector from target to camera rel to target coordinate system
        final Transform2d transform = getAccTarget(target);

        // vector from field origin to camera rel to field coordinate system
        final Translation2d coord = AprilTag.getTranslation().plus(transform.getTranslation().rotateBy(AprilTag.getRotation()));

        // angle of the camera rel to field coordinate system
        final Rotation2d angle = AprilTag.getRotation().plus(transform.getRotation());

        // turn field origin to camera vector to field origin to robot vector 
        return new Pose2d(coord, angle).transformBy(camera.offset);
    }

    /**
     * @return The name of the camera.
     */
    public String getName() {
        return camera.name;
    }
}