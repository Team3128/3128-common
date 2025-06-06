package frc.common3128.core.swerve;

/**
 * Team 3128's conversion class for Swerve
 * @since 2022 Rapid React
 * @author Mika Okamoto, Mason Lam
 */
public class SwerveConversions {
    
    /**
     * @param rotations Rotations
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double rotationsToDegrees(double rotations, double gearRatio) {
        return rotations * 360.0 / gearRatio;
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Rotations
     */
    public static double degreesToRotations(double degrees, double gearRatio) {
        double rotations = degrees / 360.0 * gearRatio;
        return rotations;
    }

    /**
     * @param rotations rotations of motor
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between motor and mechanism
     * @return Meters traveled
     */
    public static double rotationsToMeters(double rotations, double circumference, double gearRatio){
        double mechRotations = rotations / gearRatio;
        return mechRotations * circumference;
    }

    /**
     * @param meters Meters traveled
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between motor and Mechanism
     * @return rotations of motor
     */
    public static double metersToRotations(double meters, double circumference, double gearRatio){
        double mechRotations = meters / circumference;
        double rotations = mechRotations * gearRatio;
        return rotations; 
    }

    /**
     * @param RPM RPM of motor
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between motor and Mechanism
     * @return Velocity MPS
     */
    public static double RPMToMPS(double RPM, double circumference, double gearRatio){
        double wheelMPM = rotationsToMeters(RPM, circumference, gearRatio);
        double wheelMPS = wheelMPM / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between motor and Mechanism
     * @return RPM of motor
     */
    public static double MPSToRPM(double velocity, double circumference, double gearRatio){
        double wheelRPS = metersToRotations(velocity, circumference, gearRatio);
        double wheelRPM = wheelRPS * 60;
        return wheelRPM;
    }
}
