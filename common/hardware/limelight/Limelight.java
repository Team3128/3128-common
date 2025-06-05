package common.hardware.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Software wrapper to obtain data from and send data to the physical Limelight.
 * 
 * LIMELIGHT CONVENTIONS: - When the target is right of the vertical centerline,
 * tx is positive. - When the target is above the horizontal centerline, ty is
 * positive.
 * 
 * 
 * @author Adham Elarabawy, Mason Holst, Jude Lifset, Mika Okamoto
 *
 */
public class Limelight {
    public String hostname;
    public double cameraAngle;
    public double cameraHeight;
    public double targetWidth;

    public double frontDistance;
    public double robotCenterDistance;

    public NetworkTable limelightTable;

    /**
     * 
     * @param cameraAngle   - The vertical angle of the limelight. Leaning backwards is positive degrees.
     * @param cameraHeight  - The height off of the ground of the limelight lens
     * @param frontDistance - The distance between the front of the robot and the Limelight Lens
     * @param robotCenterDistance - The distance between the center of the robot and the Limelight Lens
     */
    public Limelight(String hostname, double cameraAngle, double cameraHeight, double frontDistance, double robotCenterDistance) {
        this.hostname = hostname;

        this.cameraAngle = cameraAngle;
        this.cameraHeight = cameraHeight;

        this.frontDistance = frontDistance;
        this.robotCenterDistance = robotCenterDistance;

        limelightTable = NetworkTableInstance.getDefault().getTable(hostname);
    }

    /**
     * Gets the median value of the data value in a certain key output by the
     * Limelight.
     * Switched from average to median to attempt to mitigate outliers
     * 
     * @param key        - the LimelightKey corresponding to the desired value.
     * @return
     */
    public double getValue(LimelightKey key) {
        return limelightTable.getEntry(key.getKey()).getDouble(0.0);
    }

    public double getValueAverage(LimelightKey key, int numSamples) {
        double runningTotal = 0;

        for (int i = 0; i < numSamples; i++) {
            runningTotal += getValue(key);
        }

        return runningTotal / numSamples;
    }

    /**
     * Checks to see if the Limelight has a valid target
     */
    public boolean hasValidTarget() {
        return getValue(LimelightKey.VALID_TARGET) > 0.99;
    }

    public double calculateDistToTopTarget(double targetHeight) {
        if (!hasValidTarget())
            return -1;
        double ty = getValue(LimelightKey.VERTICAL_OFFSET) * Math.PI / 180;
        double tx = getValue(LimelightKey.HORIZONTAL_OFFSET) * Math.PI / 180;
        return (targetHeight - cameraHeight) / (Math.tan(ty + cameraAngle) * Math.cos(tx)) - frontDistance;
    }
    
    public double calculateDistToGroundTarget(double targetHeight) {
        if (!hasValidTarget())
            return -1;
        double ty = getValue(LimelightKey.VERTICAL_OFFSET) * Math.PI / 180;
        return (-targetHeight + cameraHeight) * Math.tan(ty + cameraAngle) - frontDistance;
    }

    public Translation2d getTargetFieldPosition(double targetHeight, Pose2d robotPose) {
        if(!hasValidTarget()) return null;
        double ty = getValue(LimelightKey.VERTICAL_OFFSET);
        double tx = getValue(LimelightKey.HORIZONTAL_OFFSET);
        double calcDistance = (targetHeight - cameraHeight) / Math.tan(Units.degreesToRadians(ty + cameraAngle));
        Translation2d relativeTranslation = new Translation2d(calcDistance, Rotation2d.fromDegrees(tx + robotPose.getRotation().getDegrees()));
        return robotPose.getTranslation().plus(relativeTranslation);
    }

    public void setLEDMode(LEDMode mode) {
        limelightTable.getEntry("ledMode").setNumber(mode.getLEDMode());
    }

    public void setStreamMode(StreamMode mode) {
        limelightTable.getEntry("stream").setNumber(mode.getStream());
    }

    public void setPipeline(Pipeline pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline.getPipeline());
    }

      /**
     * Set the limelight on the dashboard
     */
    public double getSelectedPipeline() {
        return limelightTable.getEntry("pipeline").getDouble(0);
    }
}
