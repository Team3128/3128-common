package common.core.swerve;

import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import common.utility.shuffleboard.NAR_Shuffleboard;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public abstract class SwerveBase extends SubsystemBase {

    protected boolean fieldRelative = true;

    private double throttle = 1.0;


public final CTRESwerveDrivetrain drivetrain;

    //requests change values everytime drive() happens

    protected final SwerveRequest.FieldCentric fieldCentricRequest =
        new SwerveRequest.FieldCentric();

    protected final SwerveRequest.RobotCentric robotCentricRequest =
        new SwerveRequest.RobotCentric();

    protected final SwerveRequest.SwerveDriveBrake brakeRequest =
        new SwerveRequest.SwerveDriveBrake();

    protected final SwerveRequest.PointWheelsAt pointWheelsRequest =
        new SwerveRequest.PointWheelsAt();

    protected final SwerveRequest.Idle idleRequest = 
        new SwerveRequest.Idle();
 

public SwerveBase(CTRESwerveDrivetrain drivetrain) {
    this(drivetrain, 0, 0);
}

//maxTranslationSpeed (m/s) and maxRotationRate (rad/s) are used to set the FieldCentric/RobotCentric
//request deadbands to 10% of max, matching CTRE's generated swerve template. Without this, the
//closed-loop drive controller has zero tolerance around a 0 setpoint and will chase tiny residual
//noise (CAN latency, coupling ratio coupling, sensor quantization), causing the drive wheels to
//pulse/twitch with no controller input.
public SwerveBase(CTRESwerveDrivetrain drivetrain, double maxTranslationSpeed, double maxRotationRate) {
    this.drivetrain = drivetrain;
    fieldCentricRequest
        .withDeadband(maxTranslationSpeed * 0.1)
        .withRotationalDeadband(maxRotationRate * 0.1);
    robotCentricRequest
        .withDeadband(maxTranslationSpeed * 0.1)
        .withRotationalDeadband(maxRotationRate * 0.1);
}


    public void initShuffleboard() {
    NAR_Shuffleboard.addData(
        "Swerve",
        "CANcoder 0",
        () -> drivetrain.getModule(0)
            .getEncoder()
            .getAbsolutePosition()
            .getValue()
            .in(edu.wpi.first.units.Units.Degrees),
        0, 0
    );

    NAR_Shuffleboard.addData(
        "Swerve",
        "CANcoder 1",
        () -> drivetrain.getModule(1)
            .getEncoder()
            .getAbsolutePosition()
            .getValue()
            .in(edu.wpi.first.units.Units.Degrees),
        0, 1
    );

    NAR_Shuffleboard.addData(
        "Swerve",
        "CANcoder 2",
        () -> drivetrain.getModule(2)
            .getEncoder()
            .getAbsolutePosition()
            .getValue()
            .in(edu.wpi.first.units.Units.Degrees),
        0, 2
    );

    NAR_Shuffleboard.addData(
        "Swerve",
        "CANcoder 3",
        () -> drivetrain.getModule(3)
            .getEncoder()
            .getAbsolutePosition()
            .getValue()
            .in(edu.wpi.first.units.Units.Degrees),
        0, 3
    );

    NAR_Shuffleboard.addData(
        "Swerve",
        "Angle Motor 0",
        () -> drivetrain.getState().ModuleStates[0].angle.getDegrees(),
        1, 0
    );

    NAR_Shuffleboard.addData(
        "Swerve",
        "Angle Motor 1",
        () -> drivetrain.getState().ModuleStates[1].angle.getDegrees(),
        1, 1
    );

    NAR_Shuffleboard.addData(
        "Swerve",
        "Angle Motor 2",
        () -> drivetrain.getState().ModuleStates[2].angle.getDegrees(),
        1, 2
    );

    NAR_Shuffleboard.addData(
        "Swerve",
        "Angle Motor 3",
        () -> drivetrain.getState().ModuleStates[3].angle.getDegrees(),
        1, 3
    );

    NAR_Shuffleboard.addData(
        "Swerve",
        "Drive Motor 0",
        () -> drivetrain.getState().ModuleStates[0].speedMetersPerSecond,
        2, 0
    );

    NAR_Shuffleboard.addData(
        "Swerve",
        "Drive Motor 1",
        () -> drivetrain.getState().ModuleStates[1].speedMetersPerSecond,
        2, 1
    );

    NAR_Shuffleboard.addData(
        "Swerve",
        "Drive Motor 2",
        () -> drivetrain.getState().ModuleStates[2].speedMetersPerSecond,
        2, 2
    );

    NAR_Shuffleboard.addData(
        "Swerve",
        "Drive Motor 3",
        () -> drivetrain.getState().ModuleStates[3].speedMetersPerSecond,
        2, 3
    );

    NAR_Shuffleboard.addData(
        "Swerve",
        "Pose",
        () -> getPose().toString(),
        3, 0, 4, 1
    );

    NAR_Shuffleboard.addData(
        "Swerve",
        "Robot Velocity",
        () -> getRobotVelocity().toString(),
        3, 1, 4, 1
    );

    NAR_Shuffleboard.addData(
        "Swerve",
        "Velocity",
        () -> getSpeed(),
        3, 3
    );

    NAR_Shuffleboard.addData(
        "Swerve",
        "Angular Velocity",
        () -> getRobotVelocity().omegaRadiansPerSecond,
        5, 3
    );

    NAR_Shuffleboard.addData(
        "Swerve",
        "Field Velocity",
        () -> getFieldVelocity().toString(),
        3, 2, 4, 1
    );

    NAR_Shuffleboard.addData(
        "Swerve",
        "Gyro",
        () -> getYaw(),
        7, 0, 2, 2
    ).withWidget("Gyro");

    NAR_Shuffleboard.addCommand(
        "Swerve",
        "Reset Gyro",
        runOnce(() -> resetGyro(0)),
        7, 2
    );

    NAR_Shuffleboard.addCommand(
        "Swerve",
        "Identify Offsets",
        identifyOffsetsCommand(),
        7, 3
    );
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
    public Command identifyOffsetsCommand() {
    return Commands.runOnce(() -> {
        double[] angles = drivetrain.getRawCancoderAngles();

        for (int i = 0; i < angles.length; i++) {
            System.out.println(
                "public static final double MOD" + i +
                "_CANCODER_OFFSET = " + angles[i] + ";"
            );
        }
    });
}
}