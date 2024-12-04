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
    protected double dtConstant = 0.009;
    protected double throttle = 1;

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
        NAR_Shuffleboard.addData("Swerve", "Velocity", ()-> getSpeed(), 3, 3, 1, 1);
        NAR_Shuffleboard.addData("Swerve", "Field Velocity", ()-> getFieldVelocity().toString(), 3, 2, 4, 1);
        NAR_Shuffleboard.addData("Swerve", "Gyro", ()-> getYaw(), 7, 0, 2, 2).withWidget("Gyro");
    }

    public void drive(Translation2d translationVel, Rotation2d rotationVel) {
        drive(new ChassisSpeeds(translationVel.getX(), translationVel.getY(), rotationVel.getRadians()));
    }

    public void drive(Translation2d translationVel, double rotationVel) {
        drive(new ChassisSpeeds(translationVel.getX(), translationVel.getY(), rotationVel));
    }

    public void drive(double xVel, double yVel, double omega) {
        drive(ChassisSpeeds.fromRobotRelativeSpeeds(xVel, yVel, omega, getGyroRotation2d()));
    }

    public void drive(ChassisSpeeds velocity) {
        if(fieldRelative) velocity = ChassisSpeeds.fromRobotRelativeSpeeds(velocity, getGyroRotation2d());
        if(chassisVelocityCorrection) velocity = ChassisSpeeds.discretize(velocity, dtConstant);
        setModuleStates(kinematics.toSwerveModuleStates(velocity.times(throttle)));
        Logger.recordOutput("Swerve/DesiredModuleStates", kinematics.toSwerveModuleStates(velocity));
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

    public void setDriveVoltage(double volts) {
        for (final SwerveModule module : modules) {
            module.getDriveMotor().setVolts(volts);
        }
    }

    public Pose2d getPose() {
        return new Pose2d(estimatedPose.getTranslation(), getGyroRotation2d());
    }

    public Pose2d getRawEstimatedPose() {
        return estimatedPose;
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
        resetGyroTo(pose.getRotation().getDegrees());
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
    
    public void toggleFieldRelative() {
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
        if(NAR_Robot.logWithAdvantageKit) {
            Logger.recordOutput("Swerve/ActualModuleStates", getStates());
            Logger.recordOutput("Swerve/RobotRotation", getGyroRotation2d());
        }
    }

    public void resetAll() {
        resetOdometry(new Pose2d(0,0, new Rotation2d(0)));
        resetEncoders();
    }
    
    public void xLock() {
        modules[0].getAngleMotor().set(45, Control.Position);
        modules[1].getAngleMotor().set(-45, Control.Position);
        modules[2].getAngleMotor().set(135, Control.Position);
        modules[3].getAngleMotor().set(-135, Control.Position);
    }

    public void oLock() {
        modules[0].getAngleMotor().set(135, Control.Position);
        modules[1].getAngleMotor().set(45, Control.Position);
        modules[2].getAngleMotor().set(-135, Control.Position);
        modules[3].getAngleMotor().set(-45, Control.Position);
    }

    public void zeroLock() {
        modules[0].getAngleMotor().set(0, Control.Position);
        modules[1].getAngleMotor().set(0, Control.Position);
        modules[2].getAngleMotor().set(0, Control.Position);
        modules[3].getAngleMotor().set(0, Control.Position);
    }

    public abstract double getYaw();

    public abstract double getPitch();

    public abstract double getRoll();

    public Rotation2d getGyroRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    public abstract void resetGyroTo(double reset);

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

    public double getSpeed() {
        final ChassisSpeeds velocity = getRobotVelocity();
        return Math.hypot(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    public void initStateCheck() {
        for (final SwerveModule module : modules) {
            NarwhalDashboard.getInstance().checkState("Module" + module.moduleNumber, ()-> module.getRunningState());
        }
    }

    public Pose2d getPredictedPose(ChassisSpeeds velocity, double dt) {
        final Translation2d x = getPose().getTranslation();
        final Rotation2d theta = getPose().getRotation();
        final Translation2d dx = new Translation2d(velocity.vxMetersPerSecond * dt, velocity.vyMetersPerSecond * dt);
        final Rotation2d dtheta = Rotation2d.fromRadians(velocity.omegaRadiansPerSecond * dt);
        return new Pose2d(x.plus(dx), theta.plus(dtheta));
    }

    public Translation2d getDistanceTo(Translation2d point) {
        return getPose().getTranslation().minus(point);
    }

    public Rotation2d getAngleTo(Translation2d point) {
        return getPose().getRotation().minus(getDistanceTo(point).getAngle());
    }

}