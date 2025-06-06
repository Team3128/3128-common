package common.core.swerve;

import com.ctre.phoenix6.hardware.CANcoder;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;

/**
 * Swerve Module configuration class which is used to configure {@link SwerveModule}.
 */
public class SwerveModuleConfig {

    public static class SwerveMotorConfig {
        public final NAR_Motor motor;
        public final MotorConfig motorConfig;
        public final PIDFFConfig pidffConfig;

        public SwerveMotorConfig(NAR_Motor motor, MotorConfig motorConfig, PIDFFConfig pidffConfig) {
            this.motor = motor;
            this.motorConfig = motorConfig;
            this.pidffConfig = pidffConfig;
        }
    }

    public static class SwerveEncoderConfig {
        public final CANcoder encoder;
        public final double encoderOffset;
        public final boolean invert;

        public SwerveEncoderConfig(CANcoder encoder, double encoderOffset, boolean invert) {
            this.encoder = encoder;
            this.encoderOffset = encoderOffset;
            this.invert = invert;
        }
    }

    public final int moduleNumber;
    public final SwerveMotorConfig driveConfig;
    public final SwerveMotorConfig angleConfig;
    public final SwerveEncoderConfig encoderConfig;
    public final double maxSpeed;

    public SwerveModuleConfig(int moduleNumber, SwerveMotorConfig driveConfig, SwerveMotorConfig angleConfig, SwerveEncoderConfig encoderConfig, double maxSpeed){
        this.moduleNumber = moduleNumber;
        this.driveConfig = driveConfig;
        this.angleConfig = angleConfig;
        this.encoderConfig = encoderConfig;
        this.maxSpeed = maxSpeed;
    }
}