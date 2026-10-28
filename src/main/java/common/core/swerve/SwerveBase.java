package common.core.swerve;

import java.util.List;
import java.util.function.DoubleSupplier;

import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.shuffleboard.NAR_Shuffleboard;
import common.utility.sysid.NAR_SysIdCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Volts;

/**
 * A swerve drivetrain built from four {@link SwerveModule}s: drives the robot, and tracks where it
 * is on the field by combining wheel odometry, the gyro and vision.
 *
 * <p>Older robots used this. Since 2025 the team uses CTRE's generated swerve code
 * ({@code CommandSwerveDrivetrain}) instead, which runs odometry faster and is maintained by CTRE.
 * To use this one, extend it and fill in the gyro methods.
 *
 * <p>Speeds are in meters per second and radians per second. By default, driving is
 * field-relative: forward on the stick is away from your alliance wall, whichever way the robot
 * faces.
 */
public abstract class SwerveBase extends SubsystemBase {

    /** Whether to correct for the robot drifting sideways while turning and driving at once. */
    protected boolean chassisVelocityCorrection = true;
    /** Whether driving speeds are relative to the field (true) or to the robot's front (false). */
    public boolean fieldRelative = true;
    /** Seconds ahead the drift correction looks; tuned by hand. */
    protected double dtConstant = 0.009;
    private double throttle = 1;

    protected final SwerveDriveKinematics kinematics;
    protected SwerveDrivePoseEstimator odometry;
    protected final SwerveModule[] modules;
    private Pose2d estimatedPose;

    /** The fastest a wheel can go, in meters per second; faster requests are scaled down. */
    public double maxSpeed;

    /**
     * @param kinematics Where the four modules are on the robot
     * @param stateStdDevs How much to trust wheel odometry: bigger numbers trust it less
     * @param visionMeasurementDevs How much to trust vision: bigger numbers trust it less
     * @param configs The four modules' settings, in order 0 to 3
     */
    public SwerveBase(SwerveDriveKinematics kinematics, Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionMeasurementDevs, SwerveModuleConfig... configs) {
        this.kinematics = kinematics;
        this.maxSpeed = configs[0].maxSpeed;
        estimatedPose = new Pose2d();

        modules = new SwerveModule[] {
            new SwerveModule(configs[0]),
            new SwerveModule(configs[1]),
            new SwerveModule(configs[2]),
            new SwerveModule(configs[3])
        };
        // Timer.delay(1.5);

        resetEncoders();
        odometry = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), getPositions(),
                                                estimatedPose, stateStdDevs, visionMeasurementDevs);
    }

    /** Adds a "Swerve" Shuffleboard tab with each module's angle and speed, the pose and the gyro. */
    public void initShuffleboard() {
        for (final SwerveModule module : modules) {
            NAR_Shuffleboard.addData("Swerve", "CANcoder " + module.moduleNumber, ()-> module.getAbsoluteAngle().getDegrees(), 0, module.moduleNumber);
            NAR_Shuffleboard.addData("Swerve", "Angle Motor " + module.moduleNumber, ()-> module.getState().angle.getDegrees(), 1, module.moduleNumber);
            NAR_Shuffleboard.addData("Swerve", "Drive Motor" + module.moduleNumber, ()-> module.getState().speedMetersPerSecond, 2, module.moduleNumber);
        }
        NAR_Shuffleboard.addData("Swerve", "Pose", ()-> estimatedPose.toString(), 3, 0, 4, 1);
        NAR_Shuffleboard.addData("Swerve", "Robot Velocity", ()-> getRobotVelocity().toString(), 3, 1, 4, 1);
        NAR_Shuffleboard.addData("Swerve", "Velocity", ()-> getSpeed(), 3, 3, 1, 1);
        NAR_Shuffleboard.addData("Swerve", "Angular Velocity", () -> getRobotVelocity().omegaRadiansPerSecond, 5, 3);
        NAR_Shuffleboard.addData("Swerve", "Field Velocity", ()-> getFieldVelocity().toString(), 3, 2, 4, 1);
        NAR_Shuffleboard.addData("Swerve", "Gyro", ()-> getYaw(), 7, 0, 2, 2).withWidget("Gyro");
        NAR_Shuffleboard.addCommand("Swerve", "Reset Gyro", runOnce(()-> resetGyro(0)), 7, 2);
        NAR_Shuffleboard.addCommand("Swerve", "Identify Offsets", identifyOffsetsCommand(), 7, 3);
    }

    /** Drives at a velocity in meters per second, turning at {@code rotationVel} per second. */
    public void drive(Translation2d translationVel, Rotation2d rotationVel) {
        drive(new ChassisSpeeds(translationVel.getX(), translationVel.getY(), rotationVel.getRadians()));
    }

    /** Drives at a velocity in meters per second, turning at {@code rotationVel} radians per second. */
    public void drive(Translation2d translationVel, double rotationVel) {
        drive(new ChassisSpeeds(translationVel.getX(), translationVel.getY(), rotationVel));
    }

    // forces into robot relative speeds
    /** Drives relative to the robot's front: x forward, y left, in meters per second, and omega in radians per second. */
    public void drive(double xVel, double yVel, double omega) {
        drive(ChassisSpeeds.fromRobotRelativeSpeeds(xVel, yVel, omega, getGyroRotation2d()));
    }

    /**
     * Provides user editable insulation between requests and assignments to the swerve modules
     * @param velocity requested 
     */
    public void drive(ChassisSpeeds velocity) {
        assign(velocity);
    }

    /**
     * Assigns the requested velocity to the swerve modules
     * @param velocity requested velocity
     */
    public void assign(ChassisSpeeds velocity) {
        velocity = new ChassisSpeeds(velocity.vxMetersPerSecond * throttle, velocity.vyMetersPerSecond * throttle, velocity.omegaRadiansPerSecond);
        if(fieldRelative) velocity = ChassisSpeeds.fromFieldRelativeSpeeds(velocity, getGyroRotation2d()); // convert to field relative if applicable
        if(chassisVelocityCorrection) velocity = ChassisSpeeds.discretize(velocity, dtConstant);
        setModuleStates(kinematics.toSwerveModuleStates(velocity/*.times(throttle)*/));
    }

    /** Stops every module. */
    public void stop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    /** Sets whether the wheels brake or coast when stopped. */
    public void setBrakeMode(boolean isBrake) {
        for (final SwerveModule module : modules) {
            module.setBrakeMode(isBrake);
        }
    }

    /** Drives every wheel at a voltage, wherever it's pointing, e.g. for SysId. */
    public void setDriveVoltage(double volts) {
        for (final SwerveModule module : modules) {
            module.getDriveMotor().setVolts(volts);
        }
    }

    /** Where the robot is on the field, facing the gyro's heading. */
    public Pose2d getPose() {
        return new Pose2d(estimatedPose.getTranslation(), getGyroRotation2d());
    }

    /** Where the robot is on the field, without its heading. */
    public Translation2d getTranslation() {
        return estimatedPose.getTranslation();
    }

    /** Where the pose estimator thinks the robot is, heading included. */
    public Pose2d getRawEstimatedPose() {
        return estimatedPose;
    }

    /** Blends a vision pose into the estimate. Pass it to {@code DynamicCamera} as its consumer. */
    public void addVisionMeasurement(Pose2d pose, double timeStamp) {
        odometry.addVisionMeasurement(pose, timeStamp);
    }

    /** Points every module's angle encoder at what its CANcoder reads. */
    public void resetEncoders() {
        for (SwerveModule module : modules) {
            module.resetToAbsolute();
        }
    }

    /** Tells the robot it's at {@code pose}, e.g. at the start of an auto. */
    public void resetOdometry(Pose2d pose) {
        resetGyro(pose.getRotation().getDegrees());
        odometry.resetPosition(getGyroRotation2d(), getPositions(), pose);
    }

    /** Every module's speed and angle right now, in order 0 to 3. */
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : modules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    /** How far every wheel has rolled and which way it points, in order 0 to 3. */
    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule module : modules) {
            positions[module.moduleNumber] = module.getModulePosition();
        }
        return positions;
    }
    
    /** Switches between field-relative and robot-relative driving. */
    public void toggleFieldRelative() {
        fieldRelative = !fieldRelative;
    }

    /** Scales driving speed, e.g. 0.3 for slow, careful driving; 1 is full speed. Turning isn't scaled. */
    public void setThrottle(double throttle) {
        this.throttle = throttle;
    }

    public double getThrottle() {
        return throttle;
    }

    /** Sends each module its speed and angle, slowing them all evenly if any is over {@link #maxSpeed}. */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
        
        for (SwerveModule module : modules){
            module.setDesiredState(desiredStates[module.moduleNumber]);
        }
    }

    /** Updates the pose estimate from the wheels and gyro, every loop. */
    @Override
    public void periodic() {
        odometry.update(getGyroRotation2d(), getPositions());
        estimatedPose = odometry.getEstimatedPosition();
    }

    /** Resets the pose to the field's origin, the encoders and the gyro. */
    public void resetAll() {
        resetOdometry(new Pose2d(0,0, new Rotation2d(0)));
        resetEncoders();
        resetGyro(0);
    }
    
    /** Points the wheels in an X, so the robot is hard to push. */
    public void xLock() {
        modules[0].getAngleMotor().set(45, Control.Position);
        modules[1].getAngleMotor().set(-45, Control.Position);
        modules[2].getAngleMotor().set(135, Control.Position);
        modules[3].getAngleMotor().set(-135, Control.Position);
    }

    /** Points the wheels in a circle, so the robot spins in place. */
    public void oLock() {
        modules[0].getAngleMotor().set(135, Control.Position);
        modules[1].getAngleMotor().set(45, Control.Position);
        modules[2].getAngleMotor().set(-135, Control.Position);
        modules[3].getAngleMotor().set(-45, Control.Position);
    }

    /** Points every wheel straight ahead. */
    public void zeroLock() {
        modules[0].getAngleMotor().set(0, Control.Position);
        modules[1].getAngleMotor().set(0, Control.Position);
        modules[2].getAngleMotor().set(0, Control.Position);
        modules[3].getAngleMotor().set(0, Control.Position);
    }

    /** Points every wheel at {@code degrees}. */
    public void angleLock(double degrees) {
        for(SwerveModule module : getModules())
            module.getAngleMotor().set(degrees, Control.Position);
    }

    /** SysId tests on the drive motors; see {@code NAR_SysIdCommand}. Point the wheels first. */
    public Command characterize(double rampRate, double stepVoltage) {
        NAR_SysIdCommand characterize = new NAR_SysIdCommand(rampRate, stepVoltage, (v) -> setDriveVoltage(v.in(Volts)), this, modules[0].getDriveMotor());
       return characterize.runSysId();
    }

    /** The gyro's heading in degrees, counterclockwise positive. */
    public abstract double getYaw();

    /** The gyro's tilt forward and back, in degrees. */
    public abstract double getPitch();

    /** The gyro's tilt side to side, in degrees. */
    public abstract double getRoll();

    /** The gyro's heading. */
    public Rotation2d getGyroRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    /** Sets the gyro's heading to {@code reset} degrees. */
    public abstract void resetGyro(double reset);

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            kinematics.toChassisSpeeds(getStates()), getGyroRotation2d());
    }

    /**
     * Gets the current robot-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current robot-relative velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return kinematics.toChassisSpeeds(getStates());
    }

    /** How fast the robot is moving, in meters per second, in any direction. */
    public double getSpeed() {
        final ChassisSpeeds velocity = getRobotVelocity();
        return Math.hypot(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
    }

    /** All four modules, in order 0 to 3. */
    public SwerveModule[] getModules() {
        return modules;
    }

    /** One module, from 0 to 3. */
    public SwerveModule getModule(int moduleNumber) {
        if(moduleNumber < 0 || moduleNumber > 3) throw new IllegalArgumentException("Module number must be between 0 and 3");
        return modules[moduleNumber];
    }

    // public void initStateCheck() {
    //     for (final SwerveModule module : modules) {
    //         NarwhalDashboard.getInstance().checkState("Module" + module.moduleNumber, ()-> module.getRunningState());
    //     }
    // }

    /** Where the robot will be in {@code dt} seconds if it keeps moving at {@code velocity}. */
    public Pose2d getPredictedPose(ChassisSpeeds velocity, double dt) {
        final Translation2d x = getPose().getTranslation();
        final Rotation2d theta = getPose().getRotation();
        final Translation2d dx = new Translation2d(velocity.vxMetersPerSecond * dt, velocity.vyMetersPerSecond * dt);
        final Rotation2d dtheta = Rotation2d.fromRadians(velocity.omegaRadiansPerSecond * dt);
        return new Pose2d(x.plus(dx), theta.plus(dtheta));
    }

    /** The robot's position minus {@code point}. */
    public Translation2d getTranslation2dTo(Translation2d point) {
        return getPose().getTranslation().minus(point);
    }

    /** Meters from the robot to {@code point}. */
    public double getDistanceTo(Translation2d point) {
        return Math.abs(getPose().getTranslation().getDistance(point));
    }

    /** How far the robot would turn to face {@code point}. */
    public Rotation2d getRotation2dTo(Translation2d point) {
        return getPose().getRotation().minus(getTranslation2dTo(point).getAngle());
    }

    /** How far the robot would turn to face {@code angle}. */
    public Rotation2d getRotation2dTo(Rotation2d angle) {
        return getGyroRotation2d().minus(angle);
    }

    /** How far the robot would turn to face {@code angle}, in radians from -pi to pi. */
    public double getAngleTo(Rotation2d angle) {
        return MathUtil.angleModulus(getRotation2dTo(angle).getRadians());
    }

    /** The pose in {@code poses} closest to the robot. */
    public Pose2d nearestPose2d(List<Pose2d> poses) {
        return getPose().nearest(poses);
    }

    /** The point in {@code translations} closest to the robot. */
    public Translation2d nearestTranslation2d(List<Translation2d> translations) {
        return getPose().getTranslation().nearest(translations);
    }

    /**
     * Prints each module's CANcoder reading, ready to paste into Constants as its offset. Point every
     * wheel straight ahead first, with the bevel gears all facing the same way.
     */
    public Command identifyOffsetsCommand() {
        return runOnce(()-> {
            for(SwerveModule module : getModules()) {
                double rawAngle = module.getRawAbsoluteAngle().getDegrees();
                System.out.println("public static final double MOD" + module.moduleNumber + "_CANCODER_OFFSET = " + rawAngle + ";");
            }
        });
    }

    // public Command characterize(double startDelay, double rampRate, double targetPosition) {
    //     NAR_Motor driveMotor = modules[0].getDriveMotor();
    //     final double startPos = driveMotor.getPosition();
    //     return new CmdSysId(
    //         getName(), 
    //         (volts)-> setDriveVoltage(volts), 
    //         ()-> driveMotor.getVelocity(), 
    //         ()-> driveMotor.getPosition(), 
    //         startDelay, 
    //         rampRate, 
    //         startPos + targetPosition, 
    //         true, 
    //         this
    //     );
    // }

    // public Command characterizeTranslation(double startDelay, double rampRate, double targetPosition) {
    //     return characterize(startDelay, rampRate, targetPosition).beforeStarting(()-> angleLock(0));
    // }

    // public Command characterizeRotation(double startDelay, double rampRate, double targetPosition) {
    //     return characterize(startDelay, rampRate, targetPosition).beforeStarting(()-> oLock());
    // }

}
