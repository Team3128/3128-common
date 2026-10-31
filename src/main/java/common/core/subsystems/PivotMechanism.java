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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Rotates to an angle and holds that angle: arms, wrists, shooter hoods, turrets.
 *
 * <p>PID and feedforward run on the motor controller. The {@link Orientation} defines the direction
 * of the mechanism at 0, which determines how gravity acts:
 * <ul>
 * <li>{@link Orientation#FORWARD}, {@link Orientation#BACKWARD}, {@link Orientation#LEFT} and
 *     {@link Orientation#RIGHT} (arms, wrists, hoods) swing up and down. Angles follow WPILib: 0 is
 *     level, pointing in that direction from the pivot, and positive is up. Feedforward is kS plus kG multiplied by
 *     the cosine of the angle, so kG is the voltage required to hold the mechanism level.</li>
 * <li>{@link Orientation#TURRET} turns side to side. 0 is facing the front of the robot and
 *     positive is counterclockwise viewed from above. Gravity does not act on it, so kG should be 0.</li>
 * </ul>
 *
 * <p>The mechanism is assumed to be at {@code startPosition} when the robot turns on, and the
 * encoder is reset to it.
 *
 * <pre>{@code
 * // An arm in degrees that starts hanging straight down, can swing from -90 to 90, and weighs 4 kg
 * arm = new PivotMechanism("Arm", Orientation.FORWARD, ARM_CONFIG, ARM_GAINS, 1,
 *     -90, 90, -90, Math.PI / 180, 0.6, 4, DCMotor.getKrakenX60(1), new NAR_TalonFX(ARM_ID));
 * arm.setSetpoint(45);
 * }</pre>
 *
 * <p>For a wrist driven directly on an arm, a hood on a turret, or an arm on an elevator, see
 * {@link #setParent(MechanismBase)}.
 */
public class PivotMechanism extends MechanismBase {

    /**
     * The direction of the mechanism at 0, and therefore the axis of rotation. Each axis follows the
     * right-hand rule for a positive angle, so positive is up.
     */
    public enum Orientation {
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

    /** Acceleration due to gravity, in m/s^2. */
    private static final double GRAVITY = 9.81;

    private final Orientation orientation;
    private final double startPosition;
    private final double radiansPerUnit;
    private final double voltsPerNewtonMeter;
    private final PivotSimModel simModel;
    private Translation3d pivot = Translation3d.kZero;
    /** The pivot beneath this one that it swings with, so gravity acts on both angles; null if none. */
    private PivotMechanism gravityParent;

    /**
     * @param name Name shown on Shuffleboard and in logs
     * @param orientation The direction at 0, which determines whether gravity acts on the mechanism
     * @param motorConfig Configuration applied to every motor; its distance factor sets the position units
     * @param gains PID and feedforward gains, in volts per position unit
     * @param tolerance The distance from the setpoint within which {@link #atSetpoint()} is true
     * @param minPosition Lowest allowed setpoint
     * @param maxPosition Highest allowed setpoint
     * @param startPosition The position when the robot turns on
     * @param radiansPerUnit Radians per position unit, for example, {@code Math.PI / 180} for degrees
     * @param lengthMeters Distance from the pivot to the far end of the moving part, with any telescope retracted; used
     *                     only in simulation
     * @param massKg Moving mass, including everything mounted on it, for example, the telescope and wrist on an
     *               arm; used only in simulation
     * @param motorType Model of a single motor, for example, {@code DCMotor.getKrakenX44(1)}; used in simulation, and to
     *                  hold up the load moved out by telescopes on the mechanism
     * @param motors Every motor on the mechanism. The first motor is the leader.
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
        setDisableAtSetpoint(false);

        // distanceFactor is position units per encoder rotation, and the encoder may be on the pivot's shaft
        final double encoderRotationsPerPivotRotation = 2 * Math.PI / (motorConfig.distanceFactor * radiansPerUnit);
        final boolean gravity = orientation != Orientation.TURRET;
        if (gravity) useArmGravity(encoderRotationsPerPivotRotation);
        final double gearing = encoderRotationsPerPivotRotation * motors[0].getMotorRotationsPerSensorRotation();
        final DCMotor gearbox = MotorSimModel.gearbox(motorType, motors.length);
        // While holding still, a torque at the pivot requires this many volts
        voltsPerNewtonMeter = gearbox.rOhms / (gearing * gearbox.KtNMPerAmp);
        simModel = new PivotSimModel(gearbox, gearing, lengthMeters,
            massKg, minPosition * radiansPerUnit, maxPosition * radiansPerUnit, startPosition * radiansPerUnit,
            gravity);
        simModel.setExtraGravityVolts(this::getExtensionGravityVolts);
        motors[0].enableSim(simModel);
        reset();
    }

    /**
     * Sets the position of the pivot, in meters from the robot's origin with every mechanism at its starting
     * position, for {@link #getComponentPose()}. The value should come from CAD.
     */
    public void setPivot(Translation3d pivot) {
        this.pivot = pivot;
    }

    /**
     * Mounts this mechanism on another, for example, a wrist driven directly on an arm, an arm on an
     * elevator, or a wrist on the end of a telescoping arm. Its 3D pose moves with the parent.
     *
     * <p>On another pivot, its position is relative to the parent, with 0 aligned with the parent. When
     * it swings in the same plane as the nearest pivot beneath it, with or without a sliding
     * mechanism between them, gravity acts on the sum of their angles. The motor controller
     * knows only this mechanism's angle, so kG is calculated on the roboRIO each loop and
     * sent with the setpoint. A turret or elevator beneath it does not change gravity.
     *
     * @param parent The mechanism on which this one is mounted
     */
    @Override
    public void setParent(MechanismBase parent) {
        super.setParent(parent);

        // The nearest pivot beneath this one, through any purely sliding mechanisms
        MechanismBase below = parent;
        while (below instanceof LinearMechanism) below = below.getParent();
        if (below instanceof PivotMechanism pivotBelow && pivotBelow.orientation != Orientation.TURRET) {
            if (pivotBelow.orientation != orientation) {
                throw new IllegalArgumentException(getName() + " must have the same orientation as "
                    + pivotBelow.getName() + " to be mounted on it");
            }
            gravityParent = pivotBelow;
            useRoboRioGravity();
            simModel.setParentAngle(pivotBelow::getRadiansAboveLevel);
        }
    }

    @Override
    public boolean tiltsWhatsMountedOnIt() {
        return orientation != Orientation.TURRET || super.tiltsWhatsMountedOnIt();
    }

    /** Returns the direction of the mechanism at 0. */
    public Orientation getOrientation() {
        return orientation;
    }

    /**
     * Returns the angle above level in radians, including the angle of any parent it swings with. For a
     * turret, the angle counterclockwise from the front of the robot.
     */
    public double getRadiansAboveLevel() {
        final double own = getPosition() * radiansPerUnit;
        return gravityParent != null ? own + gravityParent.getRadiansAboveLevel() : own;
    }

    /**
     * Returns kG on the sum of this angle and the parent's, when mounted on a parent that swings in the same plane,
     * plus the voltage required to hold up the load moved out by telescopes mounted on this mechanism.
     */
    @Override
    protected double calculateFeedforward(double error) {
        // Otherwise the motor controller already applies kG
        final double kG = gravityParent != null ? gains.getkG() * Math.cos(getRadiansAboveLevel()) : 0;
        return kG + getExtensionGravityVolts();
    }

    /**
     * Returns the volts required to hold up the weight that mounted telescopes have moved out from the pivot,
     * compared with all of them retracted. kG should be tuned with every telescope retracted.
     */
    public double getExtensionGravityVolts() {
        // The axis of rotation, which tilts with the parent
        final Rotation3d tilt = getParent() == null ? Rotation3d.kZero : getParent().getComponentPose().getRotation();
        final Translation3d axis = new Translation3d(orientation.axis.get(0), orientation.axis.get(1), orientation.axis.get(2))
            .rotateBy(tilt);
        return holdingTorqueFromExtension(this, axis) * voltsPerNewtonMeter;
    }

    /** Returns the torque about {@code axis} required to hold up the load telescopes on {@code mechanism} have moved out. */
    private static double holdingTorqueFromExtension(MechanismBase mechanism, Translation3d axis) {
        double torque = 0;
        for (MechanismBase mounted : mechanism.getMounted()) {
            if (mounted instanceof LinearMechanism telescope) {
                final double weight = telescope.getMassKg() * GRAVITY;
                final Translation3d moved = telescope.getTravelDirection().times(telescope.getExtensionMeters());
                // Moving a weight by `moved` changes the torque from gravity by moved x (0, 0, -weight),
                // regardless of the pivot location; holding it requires the opposite
                torque += weight * (moved.getY() * axis.getX() - moved.getX() * axis.getY());
            }
            torque += holdingTorqueFromExtension(mounted, axis);
        }
        return torque;
    }

    /** Returns the rotation from the starting position, about the pivot from {@link #setPivot(Translation3d)}. */
    @Override
    protected Pose3d getOwnComponentPose() {
        final double radians = (getPosition() - startPosition) * radiansPerUnit;
        final Rotation3d rotation = new Rotation3d(orientation.axis, radians);
        // Rotate about the pivot rather than the robot's origin
        return new Pose3d(pivot.minus(pivot.rotateBy(rotation)), rotation);
    }
}
