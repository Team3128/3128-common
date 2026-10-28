package common.core.swerve;

import com.ctre.phoenix6.hardware.CANcoder;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;

/**
 * Everything one {@link SwerveModule} needs: its drive motor, its steering ("angle") motor, its
 * CANcoder, and its top speed.
 */
public class SwerveModuleConfig {

    /** A motor, its hardware settings, and its PID gains. */
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

    /** The CANcoder that tells the module which way its wheel points when the robot turns on. */
    public static class SwerveEncoderConfig {
        public final CANcoder encoder;
        /** Degrees the CANcoder reads with the wheel pointing straight ahead; see {@code identifyOffsetsCommand}. */
        public final double encoderOffset;
        public final boolean invert;

        public SwerveEncoderConfig(CANcoder encoder, double encoderOffset, boolean invert) {
            this.encoder = encoder;
            this.encoderOffset = encoderOffset;
            this.invert = invert;
        }
    }

    /** From 0 to 3, in the same order as the kinematics' module positions. */
    public final int moduleNumber;
    public final SwerveMotorConfig driveConfig;
    public final SwerveMotorConfig angleConfig;
    public final SwerveEncoderConfig encoderConfig;
    /** The wheel's top speed, in meters per second. */
    public final double maxSpeed;

    public SwerveModuleConfig(int moduleNumber, SwerveMotorConfig driveConfig, SwerveMotorConfig angleConfig, SwerveEncoderConfig encoderConfig, double maxSpeed){
        this.moduleNumber = moduleNumber;
        this.driveConfig = driveConfig;
        this.angleConfig = angleConfig;
        this.encoderConfig = encoderConfig;
        this.maxSpeed = maxSpeed;
    }
}