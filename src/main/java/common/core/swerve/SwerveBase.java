package common.core.swerve;

import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

//not really sure how ctre sysid works so going to leave these out for now
//import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveRotation;
//import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveTranslation;



import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public abstract class SwerveBase extends SubsystemBase {

    protected boolean fieldRelative = true;

    private double throttle = 1.0;


    protected final SwerveDrivetrain<TalonFX, TalonFX, ?> drivetrain;


    //requests change values everytime drive() happens

    protected final SwerveRequest.FieldCentric fieldCentricRequest =
        new SwerveRequest.FieldCentric();

    protected final SwerveRequest.RobotCentric robotCentricRequest =
        new SwerveRequest.RobotCentric();

    protected final SwerveRequest.SwerveDriveBrake brakeRequest =
        new SwerveRequest.SwerveDriveBrake();

    protected final SwerveRequest.PointWheelsAt pointWheelsRequest =
        new SwerveRequest.PointWheelsAt();

    public SwerveBase(
        SwerveDrivetrain<TalonFX, TalonFX, ?> drivetrain
    ) {
        this.drivetrain = drivetrain;
    }


    public void initShuffleboard() {

        //add stuff here, ctre SwerveDriveState has all the drivetrain data so need to figure out how to get that
    }

    public void drive(Translation2d translationVel, Rotation2d rotationVel) {
        drive(new ChassisSpeeds(
            translationVel.getX(),
            translationVel.getY(),
            rotationVel.getRadians()
        ));
    }

    public void drive(Translation2d translationVel, double rotationVel) {
        drive(new ChassisSpeeds(
            translationVel.getX(),
            translationVel.getY(),
            rotationVel
        ));
    }

 //drive w/robot relative velocities
    public void drive(double xVel, double yVel, double omega) {
        drive(new ChassisSpeeds(xVel, yVel, omega));
    }

    public void drive(ChassisSpeeds velocity) {
        assign(velocity);
    }

//send chassis speed request to ctre
    public void assign(ChassisSpeeds velocity) {

        velocity = new ChassisSpeeds(
            velocity.vxMetersPerSecond * throttle,
            velocity.vyMetersPerSecond * throttle,
            velocity.omegaRadiansPerSecond * throttle
        );

        if (fieldRelative) {
            drivetrain.setControl(
                fieldCentricRequest
                    .withVelocityX(velocity.vxMetersPerSecond)
                    .withVelocityY(velocity.vyMetersPerSecond)
                    .withRotationalRate(velocity.omegaRadiansPerSecond)
            );
        } else {
            drivetrain.setControl(
                robotCentricRequest
                    .withVelocityX(velocity.vxMetersPerSecond)
                    .withVelocityY(velocity.vyMetersPerSecond)
                    .withRotationalRate(velocity.omegaRadiansPerSecond)
            );
        }
    }

    public void stop() {
        drivetrain.setControl(new SwerveRequest.Idle());
    }


    public void setBrakeMode(boolean isBrake) {
        drivetrain.configNeutralMode(
            isBrake
                ? com.ctre.phoenix6.signals.NeutralModeValue.Brake
                : com.ctre.phoenix6.signals.NeutralModeValue.Coast
        );
    }


    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }

    public Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public Pose2d getRawEstimatedPose() {
        return drivetrain.getState().Pose;
    }

//preserved this from old swervebase since im not sure whether or not we need it, maybe remove later idk
    public void addVisionMeasurement(Pose2d pose, double timeStamp) {
        drivetrain.addVisionMeasurement(pose, timeStamp);
    }


    public void resetOdometry(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

//module states from ctre
    public SwerveModuleState[] getStates() {
        return drivetrain.getState().ModuleStates;
    }

    public SwerveModulePosition[] getPositions() {
        return drivetrain.getState().ModulePositions;
    }

    public void toggleFieldRelative() {
        fieldRelative = !fieldRelative;
    }

    public void setThrottle(double throttle) {
        this.throttle = throttle;
    }

    public double getThrottle() {
        return throttle;
    }


    //ctre does chassis speed to module state conversion and optimization so we dont need this for swervedrivekinematics anymore
    public void setModuleStates(SwerveModuleState[] desiredStates) {
     
        //dont implement this with old swervemodule class, if we need it later use ctre native module apis
    }

    @Override
    public void periodic() {
       //ctre does its own odometry so nothing needed here rn
    }

    public void resetAll() {
        drivetrain.resetPose(
            new Pose2d(0, 0, new Rotation2d())
        );
    }

// ctre native xlock is kind of different so i think this will work
    public void xLock() {
        drivetrain.setControl(
            brakeRequest
        );
    }

//points all wheels to supplied direcution
    public void angleLock(double degrees) {
        drivetrain.setControl(
            pointWheelsRequest
                .withModuleDirection(Rotation2d.fromDegrees(degrees))
        );
    }

    public void zeroLock() {
        angleLock(0);
    }


 public double getYaw() {
    return drivetrain.getPigeon2()
        .getYaw()
        .getValue()
        .in(edu.wpi.first.units.Units.Degrees);
}

    public double getPitch() {
        return drivetrain.getPigeon2()
            .getPitch()
            .getValue()
            .in(edu.wpi.first.units.Units.Degrees);
    }

    
    public double getRoll() {
        return drivetrain.getPigeon2()
            .getRoll()
            .getValue()
            .in(edu.wpi.first.units.Units.Degrees);
    }

    public Rotation2d getGyroRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }


    public void resetGyro(double reset) {
        drivetrain.resetRotation(
            Rotation2d.fromDegrees(reset)
        );
    }

//field relative velocity
    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            getRobotVelocity(),
            getGyroRotation2d()
        );
    }
//robot relative velocity
    public ChassisSpeeds getRobotVelocity() {
        return drivetrain.getState().Speeds;
    }

    public double getSpeed() {
        ChassisSpeeds velocity = getRobotVelocity();

        return Math.hypot(
            velocity.vxMetersPerSecond,
            velocity.vyMetersPerSecond
        );
    }

//replaces old getmodules()
    public SwerveDrivetrain<TalonFX, TalonFX, ?> getDrivetrain() {
        return drivetrain;
    }


    //need to check if velocity is field relative or not for this one, if not need to account for rotation and stuff
    public Pose2d getPredictedPose(ChassisSpeeds velocity, double dt) {
        final Translation2d x = getPose().getTranslation();
        final Rotation2d theta = getPose().getRotation();

        final Translation2d dx = new Translation2d(
            velocity.vxMetersPerSecond * dt,
            velocity.vyMetersPerSecond * dt
        );

        final Rotation2d dtheta =
            Rotation2d.fromRadians(
                velocity.omegaRadiansPerSecond * dt
            );

        return new Pose2d(
            x.plus(dx),
            theta.plus(dtheta)
        );
    }

    public Translation2d getTranslation2dTo(Translation2d point) {
        return getPose().getTranslation().minus(point);
    }

    public double getDistanceTo(Translation2d point) {
        return getPose()
            .getTranslation()
            .getDistance(point);
    }

    public Rotation2d getRotation2dTo(Translation2d point) {
        return getPose()
            .getRotation()
            .minus(getTranslation2dTo(point).getAngle());
    }

    public Rotation2d getRotation2dTo(Rotation2d angle) {
        return getGyroRotation2d().minus(angle);
    }

    public double getAngleTo(Rotation2d angle) {
        return MathUtil.angleModulus(
            getRotation2dTo(angle).getRadians()
        );
    }

    public Pose2d nearestPose2d(List<Pose2d> poses) {
        return getPose().nearest(poses);
    }

    public Translation2d nearestTranslation2d(
        List<Translation2d> translations
    ) {
        return getPose().getTranslation().nearest(translations);
    }
}