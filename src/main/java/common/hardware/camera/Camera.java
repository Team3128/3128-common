package common.hardware.camera;

import edu.wpi.first.math.geometry.Transform2d;

/**
 * Represents and holds the specs of a camera.
 * 
 * @since 2022 Rapid React
 * @author Mason Lam, William Yuan
 */
public class Camera {

    public final String name;

    public final Transform2d offset;

    public boolean enabled;

    /**
     * Creates a new object for a camera.
     * 
     * @param name Name of camera on photonvision.
     * @param enabled Whether or not to return results.
     * @param cameraOffset Offset from robot as a transform2d.
     */
    public Camera(String name, boolean enabled, Transform2d cameraOffset) {
        this.name = name;
        this.enabled = enabled;
        this.offset = cameraOffset;
    }

}