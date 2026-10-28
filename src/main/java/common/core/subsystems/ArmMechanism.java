package common.core.subsystems;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.sim.ArmSimModel;
import common.hardware.motorcontroller.sim.MotorSimModel;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Rotates to an angle and holds it against gravity: arms, pivots, wrists.
 *
 * <p>Angles follow WPILib: 0 is horizontal and positive is up. PID and feedforward run on the
 * motor controller. Feedforward is kS plus kG times the cosine of the angle, so kG is the
 * voltage to hold the arm level. The arm is assumed to be at {@code startPosition} when the robot
 * turns on, and the encoder is reset to it.
 */
public class ArmMechanism extends MechanismBase {

    private final double startPosition;
    private final double radiansPerUnit;
    private Translation3d pivot = Translation3d.kZero;

    /**
     * @param name Name shown on Shuffleboard and in logs
     * @param motorConfig Applied to every motor; its distance factor sets the position units
     * @param gains PID and feedforward gains, in volts per position unit
     * @param tolerance How close to the setpoint counts as {@link #atSetpoint()}
     * @param minPosition Lowest allowed setpoint
     * @param maxPosition Highest allowed setpoint
     * @param startPosition Where the arm is when the robot turns on
     * @param radiansPerUnit Radians per position unit, e.g. {@code Math.PI / 180} for degrees
     * @param lengthMeters Pivot to the end of the arm; used only in simulation
     * @param massKg Mass of the arm; used only in simulation
     * @param motorType Model of one motor, e.g. {@code DCMotor.getKrakenX44(1)}; used only in simulation
     * @param motors Every motor on the arm. The first is the leader.
     */
    public ArmMechanism(String name, MotorConfig motorConfig, PIDFFConfig gains, double tolerance,
                        double minPosition, double maxPosition, double startPosition, double radiansPerUnit,
                        double lengthMeters, double massKg, DCMotor motorType, NAR_Motor... motors) {
        super(gains, Control.Position, tolerance, startPosition, motorConfig, motors);
        this.startPosition = startPosition;
        this.radiansPerUnit = radiansPerUnit;
        setName(name);
        setInputRange(minPosition, maxPosition);
        setDisableAtSetpoint(false); // keep holding against gravity

        // distanceFactor is position units per encoder rotation, and the encoder may be on the arm's shaft
        final double encoderRotationsPerArmRotation = 2 * Math.PI / (motorConfig.distanceFactor * radiansPerUnit);
        useArmGravity(encoderRotationsPerArmRotation);
        final double gearing = encoderRotationsPerArmRotation * motors[0].getMotorRotationsPerSensorRotation();
        motors[0].enableSim(new ArmSimModel(MotorSimModel.gearbox(motorType, motors.length), gearing, lengthMeters,
            massKg, minPosition * radiansPerUnit, maxPosition * radiansPerUnit, startPosition * radiansPerUnit));
        reset();
    }

    /**
     * Sets where the arm's pivot is, in meters from the robot's origin, for
     * {@link #getComponentPose()}. Take it from CAD.
     */
    public void setPivot(Translation3d pivot) {
        this.pivot = pivot;
    }

    /**
     * How far the arm has rotated from where it starts, for AdvantageScope's 3D view. Rotates about
     * the pivot from {@link #setPivot(Translation3d)}, assuming the arm points toward the front of
     * the robot (+x) at 0 and swings in the robot's front-to-back plane.
     */
    @Override
    public Pose3d getComponentPose() {
        // Positive angles are up, which for an arm pointing forward is a negative pitch
        final Rotation3d rotation = new Rotation3d(0, -(getPosition() - startPosition) * radiansPerUnit, 0);
        // Rotate about the pivot rather than the robot's origin
        return new Pose3d(pivot.minus(pivot.rotateBy(rotation)), rotation);
    }
}
