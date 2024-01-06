package common.core.swerve;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;

/**
 * Swerve Module configuration class which is used to configure {@link SwerveModule}.
 */
public class SwerveModuleConfig {

    public static class SwerveMotorConfig {
        public final int motorID;
        public final MotorConfig motorConfig;
        public final PIDFFConfig pidffConfig;

        public SwerveMotorConfig(int motorID, MotorConfig motorConfig, PIDFFConfig pidffConfig) {
            this.motorID = motorID;
            this.motorConfig = motorConfig;
            this.pidffConfig = pidffConfig;
        }
    }

    public final int moduleNumber;
    public final SwerveMotorConfig driveConfig;
    public final SwerveMotorConfig angleConfig;
    public final int cancoderID;
    public final double angleOffset;
    public final boolean CANCoderinvert;
    public final double maxSpeed;

    public SwerveModuleConfig(int moduleNumber, SwerveMotorConfig driveConfig, SwerveMotorConfig angleConfig, int canCoderID, double angleOffset, boolean CANCoderinvert, double maxSpeed) {
        this.moduleNumber = moduleNumber;
        this.driveConfig = driveConfig;
        this.angleConfig = angleConfig;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
        this.CANCoderinvert = CANCoderinvert;
        this.maxSpeed = maxSpeed;
    }
}