package common.core.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static common.core.swerve.SwerveConstants.*;
import static common.core.swerve.SwerveConversions.*;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import common.hardware.motorcontroller.NAR_CANSparkMax;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.hardware.motorcontroller.NAR_CANSparkMax.EncoderType;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.hardware.motorcontroller.NAR_Motor.Neutral;


/**
 * Team 3128's Swerve Module class
 * @since 2022 Rapid React
 * @author Mika Okamato, Mason Lam
 */
public class SwerveModule {

    public enum ModuleType {
        FALCON,
        NEO
    }

    public final int moduleNumber;
    private final double angleOffset;
    private final NAR_Motor angleMotor;
    private final NAR_Motor driveMotor;
    private final CANCoder angleEncoder;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(driveKS, driveKV, driveKA);

    private Rotation2d lastAngle;

    /**
     * Creates a new Swerve Module object
     * @param moduleNumber The module number from 0 - 3.
     * @param moduleConstants The constants for the Swerve module, ie. motor ids.
     */
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        if (moduleType == ModuleType.NEO) {
            angleMotor = new NAR_CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless, EncoderType.Relative, angleKP, angleKI, angleKD);
            driveMotor = new NAR_CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless, EncoderType.Relative, driveKP, driveKI, driveKD);
        }
        else {
            angleMotor = new NAR_TalonFX(moduleConstants.angleMotorID, canBus, angleKP, angleKI, angleKD);
            driveMotor = new NAR_TalonFX(moduleConstants.driveMotorID, canBus, driveKP, driveKI, driveKD);
        }

        /* Angle Motor Config */
        configAngleMotor();

        /* Drive Motor Config */
        configDriveMotor();

        lastAngle = getState().angle;
    }

    /**
     * Initializes the angle motor
     */
    private void configAngleMotor(){
        if (angleMotor instanceof NAR_CANSparkMax) angleMotor.setCurrentLimit(angleLimit);
        if (angleMotor instanceof NAR_TalonFX) ((NAR_TalonFX) angleMotor).configAllSettings(CTREConfigs.swerveAngleFXConfig());
        angleMotor.setInverted(angleMotorInvert);
        angleMotor.setUnitConversionFactor(rotationsToDegrees(1, angleGearRatio));
        angleMotor.setNeutralMode(Neutral.COAST);
        angleMotor.enableContinuousInput(-180, 180);
        angleMotor.setDefaultStatusFrames();
        resetToAbsolute();
    }

    /**
     * Intializes the drive motor
     */
    private void configDriveMotor(){        
        if (driveMotor instanceof NAR_CANSparkMax) driveMotor.setCurrentLimit(driveLimit);
        if (driveMotor instanceof NAR_TalonFX) ((NAR_TalonFX) driveMotor).configAllSettings(CTREConfigs.swerveDriveFXConfig());
        driveMotor.setInverted(driveMotorInvert);
        driveMotor.setUnitConversionFactor(rotationsToMeters(1, wheelCircumference, driveGearRatio));
        driveMotor.setTimeConversionFactor(60);
        driveMotor.setNeutralMode(Neutral.COAST);
        driveMotor.resetPosition(0);
        driveMotor.setDefaultStatusFrames();
    }

    /**
     * Configures the CANCoder
     */
    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(CTREConfigs.swerveCancoderConfig());
    }

    /**
     * Changes the modules velocity and angular position to the desired state
     * @param desiredState The desired state with a velocity and angular component
     */
    public void setDesiredState(SwerveModuleState desiredState){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        setAngle(desiredState);
        setSpeed(desiredState);
    }

    /**
     * Changes the modules angular position to the desired state
     * @param desiredState The desired state with a velocity and angular component
     */
    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxSpeed * 0.025)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        angleMotor.set(angle.getDegrees(), Control.Position);
        lastAngle = angle;
    }

    /**
     * Changes the modules velocity to the desired state
     * @param desiredState The desired state with a velocity and angular component
     */
    private void setSpeed(SwerveModuleState desiredState) {
        driveMotor.set(desiredState.speedMetersPerSecond, Control.Velocity, feedforward.calculate(desiredState.speedMetersPerSecond));
    }

    /**
     * Sets the module in its x-lock angle and sets the velocity to 0
     * @param angle The desired angle of the module
     */
    public void xLock(Rotation2d angle) {
        final double desiredAngle = CTREModuleState.optimize(new SwerveModuleState(0, angle), getAngle()).angle.getDegrees();
        driveMotor.set(0, Control.Velocity);
        angleMotor.set(desiredAngle, Control.Position); 
    }

    /**
     * Stops the Swerve Module from moving
     */
    public void stop() {
        driveMotor.set(0);
        angleMotor.set(0);
    }

    /**
     * Resets the angle motor to the CANCoder position
     */
    public void resetToAbsolute(){
        final double absolutePosition = getCanCoder().getDegrees();
        angleMotor.resetPosition(absolutePosition);
    }

    /**
     * Returns the current angle of the CANCoder
     */
    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(MathUtil.inputModulus(angleEncoder.getAbsolutePosition() - angleOffset, -180, 180));
    }

    /**
     * Returns the Swerve module's state consisting of velocity and angular position
     * @return A swerve module state
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    /**
     * Returns the Swerve module's position containing the drive and angular position
     * @return A swerve module position
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getPosition(), getAngle());
    }

    /**
     * Returns the angular position of the swerve module
     * @return Angle in degrees
     */
    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(angleMotor.getPosition());
    }

    /**
     * Returns the module's linear velocity
     * @return Drive velocity in meters per second
     */
    private double getVelocity() {
        return driveMotor.getVelocity();
    }

    /**
     * Returns the module's linear distance traveled
     * @return Distance in meters
     */
    private double getPosition() {
        return driveMotor.getPosition();
    }

    /**
     * Sets the module's neutral mode
     * @param isBrake True for brake mode, false for coast
     */
    public void setBrakeMode(boolean isBrake) {
        driveMotor.setNeutralMode(isBrake ? Neutral.BRAKE : Neutral.COAST);
        angleMotor.setNeutralMode(isBrake ? Neutral.BRAKE : Neutral.COAST);
    }

    /**
     * Returns the drive motor
     * @return A NAR_Motor object
     */
    public NAR_Motor getDriveMotor() {
        return driveMotor;
    }

    /**
     * Returns the angle motor
     * @return A NAR_Motor object
     */
    public NAR_Motor getAngleMotor() {
        return angleMotor;
    }
}