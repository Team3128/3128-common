package frc.team3128.common.hardware.camera;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Team 3128's wrapper for {@link PhotonCamera}.
 * @since 2022 Rapid React
 * @author Mason Lam
 */
public class NAR_Camera extends PhotonCamera {

    public enum Pipeline {
        APRILTAG(1),
        RED(2),
        BLUE(3),
        Green(4);

        public int index;

        private Pipeline(int index) {
            this.index = index;
        }
    }

    private static final double FIELD_X_LENGTH = Units.inchesToMeters(648);

    private static final double FIELD_Y_LENGTH = Units.inchesToMeters(324);

    public final Camera camera;

    private PhotonPipelineResult result;

    private List<PhotonTrackedTarget> targets;

    private PhotonTrackedTarget bestTarget;

    private static DoubleSupplier gyro;
    
    private static BiConsumer<Pose2d,Double> updatePose;

    private static HashMap<Integer, Pose2d> AprilTags;
    private static Pose2d visionTarget;
    public static boolean multipleTargets = false;

    public static DoubleSupplier thresh;

    private Pipeline sPipeline;

    /**
     * Creates a NAR_Camera object
     * @param camera specs ie. name, offset from robot
     */
    public NAR_Camera(Camera camera) {
        this(camera, Pipeline.APRILTAG);
    }

    /**
     * Creates a NAR_Camera object
     * @param camera specs ie. name, offset from robot
     * @param sPipeline what type of pipeline to use
     */
    public NAR_Camera(Camera camera, Pipeline sPipeline) {
        super(camera.hostname);
        this.camera = camera;
        this.sPipeline = sPipeline;
        setLED(false);
        setVersionCheckEnabled(false);
    }

    /**
     * Feeds the angle of the robot
     */
    public static void setGyro(DoubleSupplier angle) {
        gyro = angle;
    }

    /**
     * Feeds the robot odometry object for vision estimates to update
     */
    public static void setOdometry(BiConsumer<Pose2d,Double> odometry) {
        updatePose = odometry;
    }

    /**
     * Sets the AprilTag positions on the field
     */
    public static void setAprilTags(HashMap<Integer, Pose2d> poses) {
        AprilTags = poses;
    }

    /**
     * Sets the reflective target position on the field.
     */
    public static void setVisionTarget(Pose2d pose) {
        visionTarget = pose;
    }

    /**
     * Allows the camera to update robot odometry
     */
    public void enable() {
        camera.updatePose = true;
    }

    /**
     * Stops the camera from updating robot odometry
     */
    public void disable() {
        camera.updatePose = false;
    }

    /**
     * Gets the latest targets and updates robot position if applicable
     */
    public void update() {
        //Returns the most recent camera frame
        result = this.getLatestResult();
        if (result.hasTargets()) {
            targets = result.getTargets();
            bestTarget = result.getBestTarget();
            //Checks if camera should update robot pose
            if (!camera.updatePose) return;
            ArrayList<Pose2d> poses = new ArrayList<Pose2d>();

            //Gets vision estimates from targets
            if (multipleTargets) {
                for (int i = 0; i < targets.size(); i ++) {
                    if (targetAmbiguity(targets.get(i)) < 0.3 && !getPos(targets.get(i)).equals(new Pose2d()))
                        poses.add(getPos(targets.get(i)));
                }
            }
            else if (!getPos().equals(new Pose2d()))poses.add(getPos());

            //Filters out bad estimates and updates robot position
            for (int i = 0; i < poses.size(); i++) {
                if (translationOutOfBounds(poses.get(i).getTranslation()))
                    return;
                updatePose.accept(poses.get(i),result.getTimestampSeconds());
            }
            return;
        }
        targets = null;
        bestTarget = null;
    }

    /**
     * Checks if the position given is inside the field.
     * @return A boolean whether or not the estimate is valid
     */
    private boolean translationOutOfBounds(Translation2d translation) {
        return translation.getX() > FIELD_X_LENGTH || translation.getX() < 0 || translation.getY() > FIELD_Y_LENGTH
                || translation.getY() < 0;
    }

    /**
     * Return the yaw of the target or its translation on the x axis.
     * @return The horizontal offset of the target
     */
    public double targetYaw() {
        return targetYaw(bestTarget);
    }

    /**
     * Return the yaw of the target or its translation on the x axis.
     * @param target the selected target
     * @return The horizontal offset of the target
     */
    private double targetYaw(PhotonTrackedTarget target) {
        return hasValidTarget() ? target.getYaw() : 0;
    }

    /**
     * Return the pitch of the target or its translation on the y axis.
     * @return The vertical offset of the target
     */
    public double targetPitch() {
        return targetPitch(bestTarget);
    }

    /**
     * Return the pitch of the target or its translation on the y axis.
     * @param target the selected target
     * @return The vertical offset of the target
     */
    private double targetPitch(PhotonTrackedTarget target) {
        return hasValidTarget() ? target.getPitch() : 0;
    }

    /**
     * Return the area of the target or the percent of the screen the target takes up.
     * @return the area of the target, a value from 0.0 to 1.0
     */
    public double targetArea() {
        return targetArea(bestTarget);
    }

    /**
     * Return the area of the target or the percent of the screen the target takes up.
     * @param target the selected target
     * @return The area of the target, a value from 0.0 to 1.0
     */
    private double targetArea(PhotonTrackedTarget target) {
        return hasValidTarget() ? target.getArea() : 0;
    }

    /**
     * Return the skew of the target (idk wtf this means - Mason)
     * @return The skew of the target
     */
    public double targetSkew() {
        return targetSkew(bestTarget);
    }

    /**
     * Return the skew of the target (idk wtf this means - Mason)
     * @param target the selected target
     * @return The skew of the target
     */
    private double targetSkew(PhotonTrackedTarget target) {
        return hasValidTarget() ? target.getSkew() : 0;
    }

    /**
     * Return the ID of the AprilTag
     * @return An AprilTag ID represented as a number.
     */
    public int targetId() {
        return targetId(bestTarget);
    }

    /**
     * Return the ID of the AprilTag
     * @param target the selected target
     * @return An AprilTag ID represented as a number.
     */
    private int targetId(PhotonTrackedTarget target) {
        return hasValidTarget() ? target.getFiducialId() : 0;
    }

    /**
     * Return the ambiguity of the target with lower being more accurate
     * @return A value from 0.0 to 1.0 representing pose accuracy
     */
    public double targetAmbiguity() {
        return targetAmbiguity(bestTarget);
    }

    /**
     * Return the ambiguity of the target with lower being more accurate
     * @param target the selected target
     * @return A value from 0.0 to 1.0 representing pose accuracy
     */
    private double targetAmbiguity(PhotonTrackedTarget target) {
        return hasValidTarget() ? target.getPoseAmbiguity() : 0;
    }

    /**
     * Return the raw position of the target relative to the camera
     * @return A transform3d of the target relative to the camera
     */
    public Transform3d getRawTarget() {
        return getRawTarget(bestTarget);
    }

    /**
     * Return the raw position of the target relative to the camera
     * @param target the selected target
     * @return A transform3d of the target relative to the camera
     */
    private Transform3d getRawTarget(PhotonTrackedTarget target) {
        return hasValidTarget() ? target.getBestCameraToTarget() : new Transform3d();
    }

    /**
     * Filters the raw target removing the 3d components and flipping rotation.
     * @return A filtered transform2d of the target relative to the camera
     */
    public Transform2d getTarget() {
        return getTarget(bestTarget);
    }
    
    /**
     * Filters the raw target removing the 3d components and flipping rotation.
     * @param target the selected target 
     * @return A filtered transform2d of the target relative to the camera
     */
    private Transform2d getTarget(PhotonTrackedTarget target) {
        if (!hasValidTarget()) return new Transform2d();
        Transform3d transform = getRawTarget(target);
        return new Transform2d(transform.getTranslation().toTranslation2d(), transform.getRotation().toRotation2d().unaryMinus());
    }

    /**
     * Corrects for photonvision pose error when viewing tag at an angle
     * @return Transform2d of the target relative to the camera
     * @deprecated Use {@link #getTest()} instead.
     */
    @Deprecated(forRemoval = true)
    public Transform2d getProcessedTarget() {
        return getProcessedTarget(bestTarget);
    }

    /**
     * Corrects for photonvision pose error when viewing tag at an angle
     * @param target the selected target
     * @return Transform2d of the target relative to the camera
     * @deprecated Use {@link #getTest(PhotonTrackedTarget target)} instead.
     */
    private Transform2d getProcessedTarget(PhotonTrackedTarget target) {
        if (!hasValidTarget() || !AprilTags.containsKey(targetId(target))) return new Transform2d();
        double hypotenuse = getAprilDistance(target);
        Rotation2d angle = getTarget().getRotation();
        double targetAngle = AprilTags.get(targetId(target)).getRotation().getDegrees();
        double deltaY = hypotenuse * Math.sin(Units.degreesToRadians(gyro.getAsDouble() + targetAngle + camera.offset.getRotation().getDegrees()));
        Transform2d vector = getTarget(target);
        return new Transform2d(new Translation2d(vector.getX(), vector.getY() - deltaY), angle);
    }

    /**
     * Corrects for photonvision pose error when viewing tag at an angle
     * @return Transform2d of the target relative to the camera
     */
    public Transform2d getTest() {
        return getTest(bestTarget);
    }

    /**
     * Corrects for photonvision pose error when viewing tag at an angle
     * @param target the selected target 
     * @return Transform2d of the target relative to the camera
     */
    private Transform2d getTest(PhotonTrackedTarget target) {
        if (!hasValidTarget() || !AprilTags.containsKey(targetId(target))) return new Transform2d();
        Rotation2d angle = getTarget().getRotation();
        double targetAngle = AprilTags.get(targetId(target)).getRotation().getDegrees();
        Transform2d vector = getTarget(target);
        double deltaY = vector.getX() * Math.tan(Units.degreesToRadians(gyro.getAsDouble() + targetAngle));
        return new Transform2d(new Translation2d(vector.getX(), vector.getY() + deltaY), angle);
    }
    
    /**
     * Returns whether or not the camera has a valid target
     * @return Boolean value representing a valid target
     */
    public boolean hasValidTarget() {
        return targets != null;
    }

    /**
     * Returns the distance in meters from the target
     * @return The distance in meters
     */
    public double getDistance() {
        return sPipeline.equals(Pipeline.APRILTAG) ? getAprilDistance(bestTarget) : getVisionDistance(bestTarget);
    }

    /**
     * Returns the distance in meters from the AprilTag
     * @return The distance in meters
     */
    private double getAprilDistance(PhotonTrackedTarget target) {
        if (!hasValidTarget()) return -1;
        Transform2d transform = getTarget(target);
        return Math.sqrt(Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2));
    }

    /**
     * Returns the distance in meters from the reflective target
     * @return The distance in meters
     */
    private double getVisionDistance(PhotonTrackedTarget target) {
        if (!hasValidTarget()) return -1;
        double ty = Units.degreesToRadians(targetPitch(target) + camera.angle);
        double tx = Units.degreesToRadians(targetYaw(target));
        return Math.abs(camera.targetHeight - camera.height) / (Math.tan(ty) * Math.cos(tx));
    }

    /**
     * Set the LED on or off
     * @param state boolean representing on or off
     */
    public void setLED(boolean state) {
        setLED(state ? VisionLEDMode.kOn : VisionLEDMode.kOff);
    }

    /**
     * Returns the position of the target relative to the field
     * @param robotPos the current position of the robot on the field
     * @return Pose2d containing the position of the target on the field
     */
    public Pose2d getTargetPos(Pose2d robotPos) {
        return sPipeline.equals(Pipeline.APRILTAG) ? getTargetPosApril(robotPos, bestTarget) : getTargetPosVision(robotPos, bestTarget);
    }

    /**
     * Returns the position of the AprilTag relative to the field
     * @param robotPos the current position of the robot on the field
     * @param target the selected AprilTag
     * @return Pose2d containing the position of the AprilTag on the field
     */
    private Pose2d getTargetPosApril(Pose2d robotPos, PhotonTrackedTarget target) {
        if (!hasValidTarget()) return new Pose2d();
        Pose2d cameraPos = robotPos.transformBy(camera.offset.inverse());
        Transform2d transform = getTarget(target).inverse();
        return cameraPos.plus(transform);
    }

    /**
     * Returns the position of the reflective target relative to the field
     * @param robotPos the current position of the robot on the field
     * @param target the selected reflective target
     * @return Pose2d containing the position of the reflective target on the field
     */
    private Pose2d getTargetPosVision(Pose2d robotPos, PhotonTrackedTarget target) {
        if (!hasValidTarget()) return new Pose2d();
        Pose2d cameraPos = robotPos.transformBy(camera.offset.inverse());
        Double distance = getVisionDistance(target);
        Double angle = cameraPos.getRotation().getRadians() - Units.degreesToRadians(targetYaw(target));
        return new Pose2d(cameraPos.getX() + distance * Math.cos(angle), cameraPos.getY() + distance * Math.sin(angle),
                new Rotation2d(angle));
    }

    /**
     * Returns the position of the robot on the field
     * @return Vision estimate of robot position
     */
    public Pose2d getPos() {
        return getPos(bestTarget);
    }

    /**
     * Returns the position of the robot on the field
     * @param target the selected target
     * @return Vision estimate of robot position
     */
    private Pose2d getPos(PhotonTrackedTarget target) {
        return sPipeline.equals(Pipeline.APRILTAG) ? getPosApril(target) : getPosVision(target);
    }

    /**
     * Returns the position of the robot on the field using an AprilTag
     * @param tag the selected AprilTag
     * @return Vision estimate of robot position
     */
    private Pose2d getPosApril(PhotonTrackedTarget tag) {
        if(!hasValidTarget() || !AprilTags.containsKey(targetId(tag))) return new Pose2d();
        Transform2d transform = getProcessedTarget(tag);
        if (!AprilTags.containsKey(targetId(tag)) || transform.getX() > 5 || Math.abs(transform.getRotation().getDegrees()) < 150) return new Pose2d();
        Pose2d target = AprilTags.get(targetId());
        if (target == null) return new Pose2d();
        Translation2d coord = target.getTranslation().plus(transform.getTranslation().rotateBy(target.getRotation()));
        Rotation2d angle = target.getRotation().plus(transform.getRotation());

        Pose2d pos = new Pose2d(coord,angle);

        return pos.transformBy(camera.offset);
    }

    /**
     * Returns the position of the robot on the field using a reflective target
     * @param target the selected reflective target
     * @return Vision estimate of robot position
     */
    private Pose2d getPosVision(PhotonTrackedTarget target) {
        if (!hasValidTarget()) return new Pose2d();
        double distance = getVisionDistance(target);
        double yaw = Units.degreesToRadians(targetYaw(target));
        Translation2d translation = new Translation2d(distance * Math.cos(yaw), distance * Math.sin(yaw));

        Transform2d transform = new Transform2d(translation,
                new Rotation2d(Units.degreesToRadians(gyro.getAsDouble())));
        Pose2d pos = visionTarget.transformBy(transform.inverse());
        return pos.transformBy(camera.offset);
    }

    /**
     * Changes the pipeline to the specified type
     * @param pipeline Pipeline type
     */
    public void setPipeline(Pipeline pipeline) {
        sPipeline = pipeline;
        setPipelineIndex(pipeline.index);
    }

    /**
     * Returns the hostname 
     * @return The name of the camera
     */
    public String get_name() {
        return camera.hostname;
    }

}