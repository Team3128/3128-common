package common.core.subsystems;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.sim.PivotSimModel;
import common.hardware.motorcontroller.sim.MotorSimModel;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Rotates to an angle and holds it: arms, wrists, shooter hoods, turrets.
 *
 * <p>PID and feedforward run on the motor controller. The {@link Orientation} says which way it
 * points at 0, which decides how gravity acts:
 * <ul>
 * <li>{@link Orientation#FORWARD}, {@link Orientation#BACKWARD}, {@link Orientation#LEFT} and
 *     {@link Orientation#RIGHT} (arms, wrists, hoods) swing up and down. Angles follow WPILib: 0 is
 *     level, pointing that way from the pivot, and positive is up. Feedforward is kS plus kG times
 *     the cosine of the angle, so kG is the voltage to hold it level.</li>
 * <li>{@link Orientation#TURRET} turns side to side. 0 is facing the front of the robot and
 *     positive is counterclockwise seen from above. Gravity doesn't act on it, so leave kG at 0.</li>
 * </ul>
 *
 * <p>The mechanism is assumed to be at {@code startPosition} when the robot turns on, and the
 * encoder is reset to it.
 *
 * <p>For a wrist driven directly on an arm, or a hood on a turret, see {@link #setParent(PivotMechanism)}.
 */
public class PivotMechanism extends MechanismBase {

    /** Which way the mechanism points at 0, and so the axis it rotates about. */
    public enum Orientation {
        // Each axis is the one a positive angle rotates about by the right-hand rule, so positive is up
        /** Swings up and down over the front of the robot. */
        FORWARD(0, -1, 0),
        /** Swings up and down over the back of the robot. */
        BACKWARD(0, 1, 0),
        /** Swings up and down over the left side of the robot. */
        LEFT(1, 0, 0),
        /** Swings up and down over the right side of the robot. */
        RIGHT(-1, 0, 0),
        /** Turns side to side about a vertical axis, with no gravity load. */
        TURRET(0, 0, 1);

        private final Vector<N3> axis;

        Orientation(double x, double y, double z) {
            axis = VecBuilder.fill(x, y, z);
        }
    }

    private final Orientation orientation;
    private final double startPosition;
    private final double radiansPerUnit;
    private final PivotSimModel simModel;
    private Translation3d pivot = Translation3d.kZero;
    private PivotMechanism parent;
    private boolean gravityFromParent = false;

    /**
     * @param name Name shown on Shuffleboard and in logs
     * @param orientation Which way it points at 0, which decides whether gravity acts on it
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
    public PivotMechanism(String name, Orientation orientation, MotorConfig motorConfig, PIDFFConfig gains, double tolerance,
                          double minPosition, double maxPosition, double startPosition, double radiansPerUnit,
                          double lengthMeters, double massKg, DCMotor motorType, NAR_Motor... motors) {
        super(gains, Control.Position, tolerance, startPosition, motorConfig, motors);
        this.orientation = orientation;
        this.startPosition = startPosition;
        this.radiansPerUnit = radiansPerUnit;
        setName(name);
        setInputRange(minPosition, maxPosition);
        setDisableAtSetpoint(false); // keep holding position

        // distanceFactor is position units per encoder rotation, and the encoder may be on the pivot's shaft
        final double encoderRotationsPerPivotRotation = 2 * Math.PI / (motorConfig.distanceFactor * radiansPerUnit);
        // Gravity acts on anything that swings up and down
        final boolean gravity = orientation != Orientation.TURRET;
        if (gravity) useArmGravity(encoderRotationsPerPivotRotation);
        final double gearing = encoderRotationsPerPivotRotation * motors[0].getMotorRotationsPerSensorRotation();
        simModel = new PivotSimModel(MotorSimModel.gearbox(motorType, motors.length), gearing, lengthMeters,
            massKg, minPosition * radiansPerUnit, maxPosition * radiansPerUnit, startPosition * radiansPerUnit,
            gravity);
        motors[0].enableSim(simModel);
        reset();
    }

    /**
     * Sets where the pivot is, in meters from the robot's origin with everything at its starting
     * position, for {@link #getComponentPose()}. Take it from CAD.
     */
    public void setPivot(Translation3d pivot) {
        this.pivot = pivot;
    }

    /**
     * Mounts this on another pivot, e.g. a wrist driven directly on an arm. Its position is then
     * relative to the parent, with 0 lined up with it, and its 3D pose moves with the parent's.
     *
     * <p>When both swing up and down the same way, gravity acts on the sum of their angles. The
     * motor controller only knows this mechanism's angle, so kG is worked out on the roboRIO each
     * loop instead and sent with the setpoint. On a turret, gravity isn't affected.
     *
     * @param parent What it's mounted on: a pivot with the same orientation, or a turret
     */
    public void setParent(PivotMechanism parent) {
        final boolean sameWay = parent.orientation == orientation && orientation != Orientation.TURRET;
        if (!sameWay && parent.orientation != Orientation.TURRET) {
            throw new IllegalArgumentException(getName() + " must have the same orientation as " + parent.getName()
                + " to be mounted on it");
        }
        this.parent = parent;
        if (sameWay) {
            gravityFromParent = true;
            useRoboRioGravity();
            simModel.setParentAngle(parent::getRadiansAboveLevel);
        }
    }

    /**
     * The angle above level in radians, including the angle of any parent it swings with. For a
     * turret, the angle counterclockwise from the front of the robot.
     */
    public double getRadiansAboveLevel() {
        final double own = getPosition() * radiansPerUnit;
        return gravityFromParent ? own + parent.getRadiansAboveLevel() : own;
    }

    /** kG on the parent's angle plus this one's, when mounted on a parent that swings the same way. */
    @Override
    protected double calculateFeedforward(double error) {
        // Otherwise the motor controller already applies kG
        return gravityFromParent ? gains.getkG() * Math.cos(getRadiansAboveLevel()) : 0;
    }

    /**
     * How far it has rotated from where it starts, for AdvantageScope's 3D view, rotating about
     * the pivot from {@link #setPivot(Translation3d)} and moving with any parent.
     */
    @Override
    public Pose3d getComponentPose() {
        final double radians = (getPosition() - startPosition) * radiansPerUnit;
        final Rotation3d rotation = new Rotation3d(orientation.axis, radians);
        // Rotate about the pivot rather than the robot's origin
        final Pose3d pose = new Pose3d(pivot.minus(pivot.rotateBy(rotation)), rotation);
        // Then move with the parent, as if attached to it
        return parent == null ? pose : parent.getComponentPose().transformBy(new Transform3d(pose.getTranslation(), pose.getRotation()));
    }
}
