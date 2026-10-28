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

        startSimulation(robotToCam, tagLayout);
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

        // Facing straight ahead until the first simulation update moves it with the turret
        startSimulation(new Transform3d(offset, new Rotation3d()).plus(pivotToCam), tagLayout);
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

                // prefer the multi-tag estimate, single-tag is only a fallback
                curEst = dynamicCamera.poseEstimator.estimateCoprocMultiTagPose(result);
                if (curEst.isEmpty()) {
                    curEst = dynamicCamera.poseEstimator.estimateLowestAmbiguityPose(result);
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
