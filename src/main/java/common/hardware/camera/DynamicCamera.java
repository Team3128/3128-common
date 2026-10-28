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
 * A PhotonVision camera that spots AprilTags and works out where the robot is on the field.
 *
 * <p>Every loop, {@link #periodic()} reads each camera's new results, throws out targets that are
 * too far, too close or too uncertain, works out the robot's position, and passes it to the
 * drivetrain to blend into its odometry:
 *
 * <pre>{@code
 * frontCamera = new DynamicCamera("FRONT",
 *     new Transform3d(0.3, 0, 0.5, new Rotation3d(0, Math.toRadians(-20), 0)), // where it is on the robot
 *     AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
 *     (pose, time) -> swerve.addVisionMeasurement(pose, time));
 * }</pre>
 *
 * <p>A camera on a turret is "dynamic": use the constructor that takes the turret's position and
 * angle, and it follows the turret as it turns.
 *
 * <p>In simulation, each camera sees simulated AprilTags from wherever the robot is (see
 * {@link #updateSimulation(Pose2d)}), so vision code can be tested without a robot.
 */
public class DynamicCamera {

    /** Every camera created; {@link #periodic()} reads them all. */
    public static final LinkedList<DynamicCamera> dynamicCameras = new LinkedList<DynamicCamera>();
    /** Whether {@link #periodic()} reads the cameras at all. */
    public static boolean enabled = true;
    /** Whether the last picture had any target that passed {@link #setThresholds}. */
    public boolean withinDist = true;
    /** Where this camera last put the robot. */
    public Pose2d estimatedPose = new Pose2d();
    /** Where this camera was on the field for that estimate. */
    public Pose3d cameraPose = new Pose3d();
    /** Meters to the nearest target that passed {@link #setThresholds} in the last picture. */
    public double distance = 0;
    /** Set to true to ignore just this camera, e.g. if it's knocked out of place. */
    public boolean isDisabled = false;

    /**
     * How simulated cameras see: resolution, field of view, noise, frame rate and latency. Roughly
     * an OV9281; change it before creating cameras to match yours.
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
    /** Gets each pose estimate and when it was taken, e.g. the drivetrain's addVisionMeasurement. */
    private final BiConsumer<Pose2d, Double> estConsumer;
    /** Whether it's on a turret, so it moves relative to the robot. */
    private final boolean isDynamic;
    // For a camera on a turret: from the turret's pivot to the camera, the robot's center to the
    // pivot, and the turret's angle
    private Transform3d pivotToCam;
    private Translation3d offset;
    private Supplier<Rotation2d> angularOffset;
    // Targets outside these limits are thrown out; distances in meters
    private double minDistThreshold = 0, maxDistThreshold = 100, ambiguityThreshold = 0.2;

    /**
     * A camera fixed to the robot.
     *
     * @param cameraName Its name in PhotonVision
     * @param robotToCam Where it is from the robot's center (meters, x forward, y left, z up), and
     *                   which way it points, e.g. pitched down 20 degrees
     * @param tagLayout Where the AprilTags are on this year's field
     * @param estConsumer Gets each pose estimate and its timestamp, e.g. the drivetrain's
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
     * A camera on a turret, so it turns with it.
     *
     * @param cameraName Its name in PhotonVision
     * @param pivotToCam Where it is from the turret's pivot, and which way it points, with the
     *                   turret facing forward
     * @param tagLayout Where the AprilTags are on this year's field
     * @param estConsumer Gets each pose estimate and its timestamp
     * @param offset Where the turret's pivot is from the robot's center, in meters
     * @param angularOffset The turret's angle right now, counterclockwise from facing forward
     */
    public DynamicCamera(String cameraName, Transform3d pivotToCam, AprilTagFieldLayout tagLayout, BiConsumer<Pose2d, Double> estConsumer, Translation3d offset, Supplier<Rotation2d> angularOffset) {
        this.photonCamera = new PhotonCamera(cameraName);
        this.poseEstimator = new PhotonPoseEstimator(tagLayout, pivotToCam);
        this.estConsumer = estConsumer;
        this.isDynamic = true;
        this.pivotToCam = pivotToCam;
        
        this.offset = offset;
        this.angularOffset = angularOffset;

        // Facing straight ahead until the first simulation update moves it with the turret
        startSimulation(new Transform3d(offset, new Rotation3d()).plus(pivotToCam), tagLayout);
        dynamicCameras.add(this);
    }

    /**
     * Reads every camera's new results and sends a pose estimate for each one that has a usable
     * target. Call it every loop, from {@code robotPeriodic()}.
     */
    public static void periodic() {
        if (!enabled) return;




        for (final DynamicCamera dynamicCamera : dynamicCameras) {
            if (dynamicCamera.isDisabled) continue;
            Optional<EstimatedRobotPose> curEst = Optional.empty();

            // A turret camera has moved since last loop
            if (dynamicCamera.isDynamic) {
                dynamicCamera.poseEstimator.setRobotToCameraTransform(dynamicCamera.getRobotToCam());
            }

            // A camera can take several pictures between loops; use each one
            for (var result : dynamicCamera.photonCamera.getAllUnreadResults()) {
                // Throw out targets too uncertain, too close or too far to trust as a single tag
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

                // Seeing several tags at once pins the robot down much better than one, so use that
                // when it can, and the clearest single tag otherwise. If every tag was thrown out
                // above, neither is used. Otherwise the multi-tag estimate, worked out on the
                // camera's coprocessor, uses every tag it saw, including ones thrown out. That's on
                // purpose: far tags still make a multi-tag estimate better, and several tags
                // together aren't ambiguous.
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
     * Sets which tags to trust. A tag far away or seen at a steep angle gives a less accurate
     * position, so it's often better to ignore it. A picture where no tag passes is ignored. When
     * at least one passes and the camera saw several tags, the estimate from all of them together
     * is used, including tags that didn't pass, since far tags still make it better.
     *
     * @param newMinDistThreshold Ignore tags closer than this, in meters
     * @param newMaxDistThreshold Ignore tags farther than this, in meters
     * @param newAmbiguityThreshold Ignore tags whose ambiguity is above this, from 0 (certain) to
     *                              1 (could be either of two positions); 0.2 is a common choice
     */
    public void setThresholds(double newMinDistThreshold, double newMaxDistThreshold, double newAmbiguityThreshold) {
        this.minDistThreshold = newMinDistThreshold;
        this.maxDistThreshold = newMaxDistThreshold;
        this.ambiguityThreshold = newAmbiguityThreshold;
    }

    /**
     * In simulation, gives this camera a PhotonVision sim camera that sees the field's AprilTags
     * from where the robot is, so everything downstream runs as it would on the robot.
     */
    private void startSimulation(Transform3d robotToCam, AprilTagFieldLayout tagLayout) {
        if (RobotBase.isReal()) return;
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(tagLayout);
        }
        // With the tag layout, it also works out multi-tag poses like the coprocessor does
        cameraSim = new PhotonCameraSim(photonCamera, simProperties, tagLayout);
        // Drawing what the camera sees is slow; turn these on to watch it in a browser
        cameraSim.enableRawStream(false);
        cameraSim.enableProcessedStream(false);
        visionSim.addCamera(cameraSim, robotToCam);
    }

    /**
     * Moves the simulated cameras to where the robot is, and lets them see the AprilTags from
     * there. Call every loop in {@code simulationPeriodic}, before {@link #periodic()} reads them.
     * The field with what they see is on SmartDashboard under {@code VisionSystemSim-main}.
     * @param robotPose Where the robot is on the field
     */
    public static void updateSimulation(Pose2d robotPose) {
        if (visionSim == null) return;
        for (final DynamicCamera dynamicCamera : dynamicCameras) {
            if (dynamicCamera.isDynamic) visionSim.adjustCamera(dynamicCamera.cameraSim, dynamicCamera.getRobotToCam());
        }
        visionSim.update(robotPose);
    }

    /** Turns vision back on for every camera. */
    public static void enable() {
        enabled = true;
    }

    /** Stops using every camera's estimates, e.g. if they're making the robot's position jump. */
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

    /** Adds a Shuffleboard tab for this camera, with its latest estimate and how far its nearest target is. */
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
