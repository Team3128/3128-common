package common.core.swerve;

import org.littletonrobotics.junction.Logger;

import common.core.misc.NAR_Robot;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SwerveBase extends SubsystemBase {

    public boolean chassisVelocityCorrection = true;

    protected final SwerveDriveKinematics kinematics;
    protected SwerveDrivePoseEstimator odometry;
    protected final SwerveModule[] modules;
    private Pose2d estimatedPose;

    public boolean fieldRelative;
    public double maxSpeed;

    public SwerveBase(SwerveDriveKinematics kinematics, Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionMeasurementDevs, SwerveModuleConfig... configs) {
        this.kinematics = kinematics;
        this.maxSpeed = configs[0].maxSpeed;
        fieldRelative = true;
        estimatedPose = new Pose2d();

        modules = new SwerveModule[] {
            new SwerveModule(configs[0]),
            new SwerveModule(configs[1]),
            new SwerveModule(configs[2]),
            new SwerveModule(configs[3])
        };
        Timer.delay(1.5);

        resetEncoders();

        odometry = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), getPositions(),
                                                estimatedPose, stateStdDevs, visionMeasurementDevs);
    }

    public void initShuffleboard() {
        for (final SwerveModule module : modules) {
            NAR_Shuffleboard.addData("Swerve", "CANcoder " + module.moduleNumber, ()-> module.getAbsoluteAngle().getDegrees(), 0, module.moduleNumber);
            NAR_Shuffleboard.addData("Swerve", "Angle Motor " + module.moduleNumber, ()-> module.getState().angle.getDegrees(), 1, module.moduleNumber);
            NAR_Shuffleboard.addData("Swerve", "Drive Motor" + module.moduleNumber, ()-> module.getState().speedMetersPerSecond, 2, module.moduleNumber);
        }
        NAR_Shuffleboard.addData("Swerve", "Pose", ()-> estimatedPose.toString(), 3, 0, 4, 1);
        NAR_Shuffleboard.addData("Swerve", "Robot Velocity", ()-> getRobotVelocity().toString(), 3, 1, 4, 1);
        NAR_Shuffleboard.addData("Swerve", "Velocity", ()-> getVelocity(), 3, 3, 1, 1);
        NAR_Shuffleboard.addData("Swerve", "Field Velocity", ()-> getFieldVelocity().toString(), 3, 2, 4, 1);
        NAR_Shuffleboard.addData("Swerve", "Gyro", ()-> getYaw(), 7, 0, 2, 2).withWidget("Gyro");
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        drive(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getGyroRotation2d())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    }

    public void drive(ChassisSpeeds velocity) {
        if (chassisVelocityCorrection) {
            double dtConstant = 0.009;
            Pose2d robotPoseVel = new Pose2d(velocity.vxMetersPerSecond * dtConstant,
                                            velocity.vyMetersPerSecond * dtConstant,
                                            Rotation2d.fromRadians(velocity.omegaRadiansPerSecond * dtConstant));
            Twist2d twistVel = PoseLog(robotPoseVel);

            velocity = new ChassisSpeeds(twistVel.dx / dtConstant, twistVel.dy / dtConstant,
                                        twistVel.dtheta / dtConstant);
        }
        setModuleStates(kinematics.toSwerveModuleStates(velocity));
        if(NAR_Robot.logWithAdvantageKit) Logger.recordOutput("Swerve/DesiredModuleStates", kinematics.toSwerveModuleStates(velocity));
    }

    /**
   * Logical inverse of the Pose exponential from 254. Taken from team 3181.
   *
   * @param transform Pose to perform the log on.
   * @return {@link Twist2d} of the transformed pose.
   */
    public static Twist2d PoseLog(final Pose2d transform) {
        final double kEps          = 1E-9;
        final double dtheta        = transform.getRotation().getRadians();
        final double half_dtheta   = 0.5 * dtheta;
        final double cos_minus_one = transform.getRotation().getCos() - 1.0;
        double       halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < kEps)
        {
        halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else
        {
        halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().getSin()) / cos_minus_one;
        }
        final Translation2d translation_part = transform.getTranslation()
                                                        .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta,
                                                                                -half_dtheta));
        return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
    }

    public void stop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    public void setBrakeMode(boolean isBrake) {
        for (final SwerveModule module : modules) {
            module.setBrakeMode(isBrake);
        }
    }

    public void setVoltage(double volts) {
        for (final SwerveModule module : modules) {
            module.getDriveMotor().setVolts(volts);
            module.getAngleMotor().set(0, Control.Position);
        }
    }

    public Pose2d getPose() {
        return new Pose2d(estimatedPose.getTranslation(), getGyroRotation2d());
    }

    public void addVisionMeasurement(Pose2d pose, double timeStamp) {
        odometry.addVisionMeasurement(pose, timeStamp);
    }

    public void resetEncoders() {
        for (SwerveModule module : modules) {
            module.resetToAbsolute();
        }
    }

    public void resetOdometry(Pose2d pose) {
        zeroGyro(pose.getRotation().getDegrees());
        odometry.resetPosition(getGyroRotation2d(), getPositions(), pose);
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : modules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule module : modules) {
            positions[module.moduleNumber] = module.getModulePosition();
        }
        return positions;
    }
    
    public void toggle() {
        fieldRelative = !fieldRelative;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
        
        for (SwerveModule module : modules){
            module.setDesiredState(desiredStates[module.moduleNumber]);
        }
    }

    @Override
    public void periodic() {
        odometry.update(getGyroRotation2d(), getPositions());
        estimatedPose = odometry.getEstimatedPosition();
        if (NAR_Robot.logWithAdvantageKit) {
            Logger.recordOutput("Swerve/ActualModuleStates", getStates());
            Logger.recordOutput("Swerve/RobotRotation", getGyroRotation2d());
        }
    }

    public void resetAll() {
        resetOdometry(new Pose2d(0,0, new Rotation2d(0)));
        resetEncoders();
    }
    
    //DON't USE RELIES ON APRIL TAG BAD ANGLE MEASUREMENT
    public Rotation2d getRotation2d() {
        return estimatedPose.getRotation();
    }

    public void xlock() {
        modules[0].xLock(Rotation2d.fromDegrees(45));
        modules[1].xLock(Rotation2d.fromDegrees(-45));
        modules[2].xLock(Rotation2d.fromDegrees(-45));
        modules[3].xLock(Rotation2d.fromDegrees(45));
    }

    public abstract double getYaw();

    public Rotation2d getGyroRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    //DONT USE THIS METHOD, it relies on the bad april tag angle measurements
    public double getHeading() {
        return getRotation2d().getDegrees();
    }

    public abstract double getPitch();

    public abstract double getRoll();

    public abstract void zeroGyro(double reset);

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        // ChassisSpeeds has a method to convert from field-relative to robot-relative speeds,
        // but not the reverse.  However, because this transform is a simple rotation, negating the
        // angle
        // given as the robot angle reverses the direction of rotation, and the conversion is reversed.
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            kinematics.toChassisSpeeds(getStates()), getGyroRotation2d().unaryMinus());
    }

    /**
     * Gets the current robot-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current robot-relative velocity
     */
    public ChassisSpeeds getRobotVelocity()
    {
        return kinematics.toChassisSpeeds(getStates());
    }

    public double getVelocity() {
        final ChassisSpeeds speeds = getRobotVelocity();
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    public void initStateCheck() {
        for (final SwerveModule module : modules) {
            NarwhalDashboard.getInstance().checkState("Module" + module.moduleNumber, ()-> module.getRunningState());
        }
    }
}