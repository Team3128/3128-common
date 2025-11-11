package common.hardware.camera;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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

/**
 * Team 3128's class to control the robot's cameras and vision processing.
 * 
 * @since 2024 Crescendo
 * @author Audrey Zheng
 */
public class Camera {

    public PhotonCamera camera;

    private PhotonPipelineResult result = new PhotonPipelineResult();
    private List<PhotonPipelineResult> resultList = new ArrayList<PhotonPipelineResult>();

    private static DoubleSupplier gyro;
    private static AprilTagFieldLayout aprilTags;
    private static BiConsumer<Pose2d, Double> odometry;
    private static Supplier<Pose2d> robotPose;

    private Transform3d robotToCamera = Transform3d.kZero;
    private PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR; 

    private double maxDistanceThreshold = 5;
    private double ambiguityThreshold = 0.5;
    private double minDistanceThreshold = 0.34;

    public static boolean isDisabled = false;

    private PhotonPoseEstimator estimator = new PhotonPoseEstimator(aprilTags, poseStrategy, robotToCamera);

    public void update() {
        for (var change : camera.getAllUnreadResults()) {
            
        }
    }
            
    public void enable() {
        isDisabled = false;
    }

    public void disable(){
        isDisabled = true;
    }

    public void initShuffleboard() {
    
    }
}
