package common.core.subsystems;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.sim.PivotSimModel;
import common.hardware.motorcontroller.sim.MotorSimModel;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Rotates to an angle and holds it: arms, wrists, shooter hoods, turrets.
 *
 * <p>PID and feedforward run on the motor controller. The {@link Axis} decides how gravity acts:
 * <ul>
 * <li>{@link Axis#PITCH} (arms, wrists, hoods) swings up and down. Angles follow WPILib: 0 is
 *     horizontal, pointing toward the front of the robot, and positive is up. Feedforward is kS
 *     plus kG times the cosine of the angle, so kG is the voltage to hold it level.</li>
 * <li>{@link Axis#YAW} (turrets) turns side to side. 0 is facing the front of the robot and
 *     positive is counterclockwise seen from above. Gravity doesn't act on it, so leave kG at 0.</li>
 * </ul>
 *
 * <p>The mechanism is assumed to be at {@code startPosition} when the robot turns on, and the
 * encoder is reset to it.
 */
public class PivotMechanism extends MechanismBase {

    /** The axis the mechanism rotates about. */
    public enum Axis {
        /** Side to side, so it swings up and down against gravity: arms, wrists, hoods. */
        PITCH,
        /** Straight up, so it turns side to side with no gravity load: turrets. */
        YAW
    }

    private final Axis axis;
    private final double startPosition;
    private final double radiansPerUnit;
    private Translation3d pivot = Translation3d.kZero;

    /**
     * @param name Name shown on Shuffleboard and in logs
     * @param axis The axis it rotates about, which decides whether gravity acts on it
     * @param motorConfig Applied to every motor; its distance factor sets the position units
     * @param gains PID and feedforward gains, in volts per position unit
     * @param tolerance How close to the setpoint counts as {@link #atSetpoint()}
     * @param minPosition Lowest allowed setpoint
     * @param maxPosition Highest allowed setpoint
     * @param startPosition Where it is when the robot turns on
     * @param radiansPerUnit Radians per position unit, e.g. {@code Math.PI / 180} for degrees
     * @param lengthMeters Pivot to the far end of the moving part; used only in simulation
     * @param massKg Mass of the moving part; used only in simulation
     * @param motorType Model of one motor, e.g. {@code DCMotor.getKrakenX44(1)}; used only in simulation
     * @param motors Every motor on the mechanism. The first is the leader.
     */
    public PivotMechanism(String name, Axis axis, MotorConfig motorConfig, PIDFFConfig gains, double tolerance,
                          double minPosition, double maxPosition, double startPosition, double radiansPerUnit,
                          double lengthMeters, double massKg, DCMotor motorType, NAR_Motor... motors) {
        super(gains, Control.Position, tolerance, startPosition, motorConfig, motors);
        this.axis = axis;
        this.startPosition = startPosition;
        this.radiansPerUnit = radiansPerUnit;
        setName(name);
        setInputRange(minPosition, maxPosition);
        setDisableAtSetpoint(false); // keep holding position

        // distanceFactor is position units per encoder rotation, and the encoder may be on the pivot's shaft
        final double encoderRotationsPerPivotRotation = 2 * Math.PI / (motorConfig.distanceFactor * radiansPerUnit);
        if (axis == Axis.PITCH) useArmGravity(encoderRotationsPerPivotRotation);
        final double gearing = encoderRotationsPerPivotRotation * motors[0].getMotorRotationsPerSensorRotation();
        motors[0].enableSim(new PivotSimModel(MotorSimModel.gearbox(motorType, motors.length), gearing, lengthMeters,
            massKg, minPosition * radiansPerUnit, maxPosition * radiansPerUnit, startPosition * radiansPerUnit,
            axis == Axis.PITCH));
        reset();
    }

    /**
     * Sets where the pivot is, in meters from the robot's origin, for {@link #getComponentPose()}.
     * Take it from CAD.
     */
    public void setPivot(Translation3d pivot) {
        this.pivot = pivot;
    }

    /**
     * How far it has rotated from where it starts, for AdvantageScope's 3D view, rotating about
     * the pivot from {@link #setPivot(Translation3d)}.
     */
    @Override
    public Pose3d getComponentPose() {
        final double radians = (getPosition() - startPosition) * radiansPerUnit;
        // Positive pitch angles are up, which for something pointing forward is a negative pitch
        final Rotation3d rotation = axis == Axis.PITCH ? new Rotation3d(0, -radians, 0) : new Rotation3d(0, 0, radians);
        // Rotate about the pivot rather than the robot's origin
        return new Pose3d(pivot.minus(pivot.rotateBy(rotation)), rotation);
    }
}
