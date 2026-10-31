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
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotBase;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * A PhotonVision camera that detects AprilTags and estimates the robot's position on the field.
 *
 * <p>Every loop, {@link #periodic()} reads each camera's new results, discards targets that are
 * too far, too close or too ambiguous, estimates the robot's position, and passes it to the
 * drivetrain to merge into its odometry:
 *
 * <pre>{@code
 * frontCamera = new DynamicCamera("FRONT",
 *     new Transform3d(0.3, 0, 0.5, new Rotation3d(0, Math.toRadians(-20), 0)), // camera position on the robot
 *     AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
 *     (pose, time) -> swerve.addVisionMeasurement(pose, time));
 * }</pre>
 *
 * <p>A camera on a turret is "dynamic": the constructor that takes the turret's position and
 * angle makes the camera follow the turret as it turns.
 *
 * <p>In simulation, each camera sees simulated AprilTags from the robot's position (see
 * {@link #updateSimulation(Pose2d)}), so vision code can be tested without a robot.
 */
public class DynamicCamera {

    /** Every camera created; {@link #periodic()} reads all of them. */
    public static final LinkedList<DynamicCamera> dynamicCameras = new LinkedList<DynamicCamera>();
    /** Whether {@link #periodic()} reads the cameras. */
    public static boolean enabled = true;
    /** Whether the last frame had any target that passed {@link #setThresholds}. */
    public boolean withinDist = true;
    /** The robot pose most recently estimated by this camera. */
    public Pose2d estimatedPose = new Pose2d();
    /** The position of this camera on the field for that estimate. */
    public Pose3d cameraPose = new Pose3d();
    /** Distance in meters to the nearest target that passed {@link #setThresholds} in the last frame. */
    public double distance = 0;
    /** Set to true to ignore this camera only, for example, if it has been knocked out of position. */
    public boolean isDisabled = false;

    /**
     * The properties of simulated cameras: resolution, field of view, noise, frame rate and latency.
     * The defaults approximate an OV9281; change them before creating cameras to match the hardware.
     */
    public static SimCameraProperties simProperties = new SimCameraProperties()
        .setCalibration(1280, 800, Rotation2d.fromDegrees(80))
        .setCalibError(0.25, 0.08)
        .setFPS(30)
        .setAvgLatencyMs(30)
        .setLatencyStdDevMs(5);
    private static VisionSystemSim visionSim;
    private PhotonCameraSim cameraSim;

    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator poseEstimator;
    /** Receives each pose estimate and its timestamp, for example, the drivetrain's addVisionMeasurement. */
    private final BiConsumer<Pose2d, Double> estConsumer;
    /** Whether the camera is on a turret and so moves relative to the robot. */
    private final boolean isDynamic;
    /** On a turret, the transform from the turret's pivot to the camera. */
    private Transform3d pivotToCam;
    /** On a turret, the translation from the robot's center to the pivot. */
    private Translation3d offset;
    /** On a turret, the turret's angle. */
    private Supplier<Rotation2d> angularOffset;
    /** Targets outside these limits are discarded. Distances are in meters. */
    private double minDistThreshold = 0, maxDistThreshold = 100, ambiguityThreshold = 0.2;

    /**
     * Creates a camera fixed to the robot.
     *
     * @param cameraName The camera's name in PhotonVision
     * @param robotToCam The camera's position from the robot's center (meters, x forward, y left, z up)
     *                   and orientation, for example, pitched down 20 degrees
     * @param tagLayout The positions of the AprilTags on this year's field
     * @param estConsumer Receives each pose estimate and its timestamp, for example, the drivetrain's
     *                    {@code addVisionMeasurement}
     */
    public DynamicCamera(String cameraName, Transform3d robotToCam, AprilTagFieldLayout tagLayout, BiConsumer<Pose2d, Double> estConsumer) {
        this.photonCamera = new PhotonCamera(cameraName);
        this.poseEstimator = new PhotonPoseEstimator(tagLayout, robotToCam);
        this.estConsumer = estConsumer;
        this.isDynamic = false;

        startSimulation(robotToCam, tagLayout);
        dynamicCameras.add(this);
    }

    /**
     * Creates a camera mounted on a turret, which turns with the turret.
     *
     * @param cameraName The camera's name in PhotonVision
     * @param pivotToCam The camera's position from the turret's pivot and its orientation, with the
     *                   turret facing forward
     * @param tagLayout The positions of the AprilTags on this year's field
     * @param estConsumer Receives each pose estimate and its timestamp
     * @param offset The position of the turret's pivot from the robot's center, in meters
     * @param angularOffset The current turret angle, counterclockwise from facing forward
     */
    public DynamicCamera(String cameraName, Transform3d pivotToCam, AprilTagFieldLayout tagLayout, BiConsumer<Pose2d, Double> estConsumer, Translation3d offset, Supplier<Rotation2d> angularOffset) {
        this.photonCamera = new PhotonCamera(cameraName);
        this.poseEstimator = new PhotonPoseEstimator(tagLayout, pivotToCam);
        this.estConsumer = estConsumer;
        this.isDynamic = true;
        this.pivotToCam = pivotToCam;
        
        this.offset = offset;
        this.angularOffset = angularOffset;

        // Faces straight ahead until the first simulation update moves it with the turret
        startSimulation(new Transform3d(offset, new Rotation3d()).plus(pivotToCam), tagLayout);
        dynamicCameras.add(this);
    }

    /**
     * Reads every camera's new results and sends a pose estimate for each one that has a usable
     * target. Call this method every loop, from {@code robotPeriodic()}.
     *
     * <p>Targets that are too ambiguous, too close or too far to trust as a single tag are
     * discarded first. If none remain, no estimate is sent. Otherwise the coprocessor's multi-tag
     * estimate is preferred, with the lowest-ambiguity single tag as the fallback.
     *
     * <p>The multi-tag estimate deliberately uses every tag the camera saw, including discarded
     * ones: distant tags still improve it, and several tags together are not ambiguous.
     */
    public static void periodic() {
        if (!enabled) return;




        for (final DynamicCamera dynamicCamera : dynamicCameras) {
            if (dynamicCamera.isDisabled) continue;
            Optional<EstimatedRobotPose> curEst = Optional.empty();

            // A turret camera may have moved since the last loop
            if (dynamicCamera.isDynamic) {
                dynamicCamera.poseEstimator.setRobotToCameraTransform(dynamicCamera.getRobotToCam());
            }

            // A camera can capture several frames between loops, and each one is used
            for (var result : dynamicCamera.photonCamera.getAllUnreadResults()) {
                double nearest = Double.POSITIVE_INFINITY;
                for (int i = 0; i < result.targets.size(); i++) {
                    final PhotonTrackedTarget target = result.targets.get(i);
                    final double distance = target.bestCameraToTarget.getTranslation().getNorm();
                    final boolean inRange = distance >= dynamicCamera.minDistThreshold && distance <= dynamicCamera.maxDistThreshold;
                    if (!inRange || target.poseAmbiguity > dynamicCamera.ambiguityThreshold) {
                        result.targets.remove(i);
                        i--;
                    } else {
                        nearest = Math.min(nearest, distance);
                    }
                }
                dynamicCamera.withinDist = !result.targets.isEmpty();
                if (dynamicCamera.withinDist) dynamicCamera.distance = nearest;

                curEst = dynamicCamera.poseEstimator.estimateCoprocMultiTagPose(result);
                if (curEst.isEmpty()) {
                    curEst = dynamicCamera.poseEstimator.estimateLowestAmbiguityPose(result);
                }

                if (curEst.isPresent()) {
                    final Pose3d robotPose = curEst.get().estimatedPose;
                    dynamicCamera.estimatedPose = robotPose.toPose2d();
                    dynamicCamera.cameraPose = robotPose.transformBy(dynamicCamera.poseEstimator.getRobotToCameraTransform());
                    dynamicCamera.estConsumer.accept(dynamicCamera.estimatedPose, curEst.get().timestampSeconds);
                }
            }
        }
    }

    /**
     * Sets which tags to trust. A distant tag or one seen at a steep angle gives a less accurate
     * position and is often better ignored. A frame in which no tag passes is ignored. When at
     * least one passes and the camera saw several tags, the estimate from all of them together is
     * used, including tags that did not pass, because distant tags still improve it.
     *
     * @param newMinDistThreshold Tags closer than this are ignored, in meters
     * @param newMaxDistThreshold Tags farther than this are ignored, in meters
     * @param newAmbiguityThreshold Tags with ambiguity above this are ignored, from 0 (certain) to
     *                              1 (either of two positions); 0.2 is a common value
     */
    public void setThresholds(double newMinDistThreshold, double newMaxDistThreshold, double newAmbiguityThreshold) {
        this.minDistThreshold = newMinDistThreshold;
        this.maxDistThreshold = newMaxDistThreshold;
        this.ambiguityThreshold = newAmbiguityThreshold;
    }

    /**
     * In simulation, gives this camera a PhotonVision sim camera that sees the field's AprilTags
     * from the robot's position, so that all downstream code runs as it would on the robot.
     */
    private void startSimulation(Transform3d robotToCam, AprilTagFieldLayout tagLayout) {
        if (RobotBase.isReal()) return;
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(tagLayout);
        }
        // With the tag layout, this also calculates multi-tag poses as the coprocessor does
        cameraSim = new PhotonCameraSim(photonCamera, simProperties, tagLayout);
        // Rendering the camera view is slow; enable these to watch it in a browser
        cameraSim.enableRawStream(false);
        cameraSim.enableProcessedStream(false);
        visionSim.addCamera(cameraSim, robotToCam);
    }

    /**
     * Moves the simulated cameras to the robot's position and updates what they see from
     * there. Call this every loop in {@code simulationPeriodic}, before {@link #periodic()}.
     * The field view is on SmartDashboard under {@code VisionSystemSim-main}.
     * @param robotPose The robot's pose on the field
     */
    public static void updateSimulation(Pose2d robotPose) {
        if (visionSim == null) return;
        for (final DynamicCamera dynamicCamera : dynamicCameras) {
            if (dynamicCamera.isDynamic) visionSim.adjustCamera(dynamicCamera.cameraSim, dynamicCamera.getRobotToCam());
        }
        visionSim.update(robotPose);
    }

    /** Enables vision for every camera. */
    public static void enable() {
        enabled = true;
    }

    /** Stops using every camera's estimates, for example, if they are making the robot's position jump. */
    public static void disable() {
        enabled = false;
    }

    /** Robot center to turret pivot, rotated by the turret angle, then pivot to camera. */
    private Transform3d getRobotToCam() {
        Transform3d robotToPivot = new Transform3d(
            offset,
            new Rotation3d(0, 0, angularOffset.get().getRadians()));
        return robotToPivot.plus(pivotToCam);
    }

    /** Adds a Shuffleboard tab for this camera, with its latest estimate and the distance to its nearest target. */
    public void initShuffleboard() {
        NAR_Shuffleboard.addData(photonCamera.getName(), "WithinDist", () -> withinDist, 2, 2, 1, 1);
        NAR_Shuffleboard.addData(photonCamera.getName(), "Estimated Pose X", () -> estimatedPose.getX(), 0, 0, 2, 1);
        NAR_Shuffleboard.addData(photonCamera.getName(), "Estimated Pose Y", () -> estimatedPose.getY(), 0, 1, 2, 1);
        NAR_Shuffleboard.addData(photonCamera.getName(), "camera pose X", () -> cameraPose.getX(), 2, 0, 2, 1);
        NAR_Shuffleboard.addData(photonCamera.getName(), "camera pose Y", () -> cameraPose.getY(), 2, 1, 2, 1);
        NAR_Shuffleboard.addData(photonCamera.getName(), "dist", () -> distance, 0, 2, 1, 1);
        NAR_Shuffleboard.addData(photonCamera.getName(), "Enabled", () -> !isDisabled, 3, 2);
    }
}
