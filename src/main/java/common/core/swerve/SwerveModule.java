package common.core.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import common.core.controllers.PIDFFConfig;
import common.core.swerve.SwerveModuleConfig.SwerveMotorConfig;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.narwhaldashboard.NarwhalDashboard.State;

import java.util.function.Supplier;


/**
 * Team 3128's Swerve Module class
 * @since 2022 Rapid React
 * @author Mika Okamato, Mason Lam
 */
public class SwerveModule {

    public static boolean shouldOptimizeCAN = true;

    public final int moduleNumber;
    private final double angleOffset;
    private final NAR_Motor angleMotor;
    private final NAR_Motor driveMotor;
    private final CANcoder angleEncoder;
    private final SwerveMotorConfig driveConfig;
    private final SwerveMotorConfig angleConfig;

    private final SimpleMotorFeedforward feedforward;

    private final double maxSpeed;

    private Rotation2d lastAngle;

    private final Supplier<Double> absoluteAngle;

    /**
     * Creates a new Swerve Module object
     * @param config Settings for the Swerve Module.
     */
    public SwerveModule(SwerveModuleConfig config){
        this.moduleNumber = config.moduleNumber;
        this.driveConfig = config.driveConfig;
        this.angleConfig = config.angleConfig;
        this.maxSpeed = config.maxSpeed;
        angleOffset = config.angleOffset;

        final PIDFFConfig drivePIDConfig = driveConfig.pidffConfig;
        feedforward = new SimpleMotorFeedforward(drivePIDConfig.kS, drivePIDConfig.kV, drivePIDConfig.kA);
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(config.cancoderID);
        var absoluteAngleSupplier = angleEncoder.getAbsolutePosition();
        if (shouldOptimizeCAN) {
            absoluteAngleSupplier.setUpdateFrequency(100);
            angleEncoder.optimizeBusUtilization();
        }
        absoluteAngle = absoluteAngleSupplier.asSupplier();
        
        
        final SensorDirectionValue direction = config.CANCoderinvert ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        angleEncoder.getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(direction));

        angleMotor = angleConfig.motor;
        driveMotor = driveConfig.motor;

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
        angleMotor.configMotor(angleConfig.motorConfig);
        angleMotor.configPID(angleConfig.pidffConfig);
        angleMotor.enableContinuousInput(-180, 180);
        angleMotor.setDefaultStatusFrames();
        resetToAbsolute();
    }

    /**
     * Intializes the drive motor
     */
    private void configDriveMotor(){        
        driveMotor.configMotor(driveConfig.motorConfig);
        driveMotor.configPID(driveConfig.pidffConfig);
        driveMotor.resetPosition(0);
        driveMotor.setDefaultStatusFrames();
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
        return Rotation2d.fromDegrees(MathUtil.inputModulus(absoluteAngle.get() * 360 - angleOffset, -180, 180));
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
    
    /**
     * Return the state of the swerve module.
     * @return State of the swerve module.
     */
    public State getRunningState() {
        if (driveMotor.getState() == State.RUNNING && angleMotor.getState() == State.RUNNING) {
            return State.RUNNING; 
        }
        if (driveMotor.getState() == State.DISCONNECTED || angleMotor.getState() == State.DISCONNECTED) {
            return State.PARTIALLY_RUNNING; 
        }
        return State.DISCONNECTED;
    }
}