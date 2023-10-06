package common.hardware.camera;

import edu.wpi.first.math.geometry.Transform2d;

/**
 * Represents and holds the specs of a camera.
 * 
 * @since 2023 Charged Up
 * @author Mason Lam, William Yuan
 */
public class Camera {

    public String hostname;

    public boolean updatePose;
  
    public double height; //inches

    public double angle;  //degrees

    public double targetHeight; //inches

    public Transform2d offset; //inches

    /**
     * Creates a new object for a camera.
     * 
     * @param hostname Name of camera.
     * @param updatePose Whether or not to update pose in a boolean.
     * @param cameraHeight Height of the camera in inches
     * @param cameraAngle Angle of the camera in degrees.
     * @param targetHeight Height of the target in inches.
     * @param cameraOffset offset from robot in inches.
     */
    public Camera(String hostname, boolean updatePose, double cameraHeight, double cameraAngle, double targetHeight, Transform2d cameraOffset) {
        this.hostname = hostname;
        this.updatePose = updatePose;
        this.height = cameraHeight;
        this.angle = cameraAngle;
        this.targetHeight = targetHeight;
        this.offset = cameraOffset;
    }

}