package common.core.controllers;

import java.util.function.Supplier;

import common.core.swerve.SwerveBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;

public class SwerveController {
    private final Supplier<Pose2d> goalPoseSupplier;
    private final SwerveBase swerve;
    private final Supplier<Translation2d> feedfowardSupplier;
    private final driveController driveController;
    private final angleController angleController;
    private final TunableDouble timeTolerance = new TunableDouble(0.5, "SwerveController", "timeTolerance", 4,0);
    private Timer toleranceTimer = new Timer();


    public SwerveController(Supplier<Pose2d> goalPoseSupplier, SwerveBase swerve, Supplier<Translation2d> feedforwardSupplier, driveController driveController, angleController angleController) {
        this.goalPoseSupplier = goalPoseSupplier;
        this.swerve = swerve;
        this.feedfowardSupplier = feedforwardSupplier;
        this.driveController = driveController;
        this.angleController = angleController;

        toleranceTimer.restart();
        resetControllers();

    }

    private void resetControllers(){
        Pose2d currentPose = swerve.getPose(); //this is sus idk if it works...i think it does
        Pose2d goalPose = goalPoseSupplier.get();
        ChassisSpeeds fieldVelocity = swerve.getFieldVelocity();
        double totalLinearVelocity = Math.hypot(fieldVelocity.vxMetersPerSecond, fieldVelocity.vxMetersPerSecond);
        double totalAngularVelocity = fieldVelocity.omegaRadiansPerSecond;
        driveController.reset();
        angleController.reset();
        driveController.setSetpoint(new State(currentPose.getTranslation().getDistance(goalPose.getTranslation()), totalLinearVelocity));
        angleController.setSetpoint(new State(currentPose.getRotation().minus(goalPose.getRotation()).getDegrees(), totalAngularVelocity));
    }

    public void update(){
        // what we wanna do is get to our goal 

            
    }
    
    public class driveController extends TrapController {

        private final TunableDouble linearTolerance = new TunableDouble(0, "SwerveController", "linearTolerance", 2, 0); //placeholder value
        // private final TunableDouble maxLinearVelocity = new TunableDouble(15.0, "SwerveController", "maxLinearVelocity", 2, 1);
        // private final TunableDouble maxLinearAcceleration = new TunableDouble(30.0, "SwerveController", "maxLinearAcceleration", 2, 2);

        public driveController(PIDFFConfig config, Constraints constraints){
            super(config, constraints);
            
            final TunableDouble linearkP = new TunableDouble(config.kP, "SwerveController", "linearkP", 0,0);
            final TunableDouble linearkI = new TunableDouble(config.kI, "SwerveController", "linearkI", 0,1);
            final TunableDouble linearkD = new TunableDouble(config.kD, "SwerveController", "linearkD", 0,2);
            
            driveController.setTolerance(linearTolerance.getAsDouble());
        }

    }

    public class angleController extends TrapController {
        
        private final TunableDouble angleTolerance = new TunableDouble(0, "SwerveController", "angleTolerance", 3, 1); //placeholder value
        // private final TunableDouble maxAngularVelocity = new TunableDouble(12.0, "SwerveController", "maxLinearVelocity", 3, 1);
        // private final TunableDouble maxAngularAcceleration = new TunableDouble(4.8, "SwerveController", "maxLinearAcceleration", 3, 2);

        public angleController(PIDFFConfig config, Constraints constraints){
            super(config, constraints);

            final TunableDouble thetakP = new TunableDouble(config.kP, "SwerveController", "thetakP", 1,0);
            final TunableDouble thetakI = new TunableDouble(config.kI, "SwerveController", "thetakI", 1,1);
            final TunableDouble thetakD = new TunableDouble(config.kD, "SwerveController", "thetakD", 1,2);

            angleController.enableContinuousInput(-Math.PI, Math.PI);
            angleController.setTolerance(angleTolerance.getAsDouble());

        }

    }
}
