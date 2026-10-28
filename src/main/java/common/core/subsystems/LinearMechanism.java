package common.core.subsystems;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.sim.LinearSimModel;
import common.hardware.motorcontroller.sim.MotorSimModel;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Moves in a straight line to a position and holds it, at any incline:
 * sliders (0 degrees), elevators (90 degrees), or anything in between.
 *
 * <p>PID and feedforward run on the motor controller. Feedforward is kS plus a constant kG
 * from {@code gains}; leave kG at 0 for a horizontal slider. The mechanism is assumed to be at {@code minPosition}
 * when the robot turns on.
 *
 * <pre>{@code
 * // A two-motor elevator measured in meters, 1.2 m of travel, carrying 8 kg
 * elevator = new LinearMechanism("Elevator", ELEVATOR_CONFIG, ELEVATOR_GAINS, 0.01,
 *     0, 1.2, 1, Math.PI / 2, 8, DCMotor.getKrakenX60(1),
 *     new NAR_TalonFX(ELEVATOR_LEFT_ID), new NAR_TalonFX(ELEVATOR_RIGHT_ID));
 * elevator.invertMotor(1); // the right motor faces the other way
 * elevator.setSetpoint(0.8);
 * }</pre>
 *
 * <p>Mounted on an arm, e.g. a telescoping arm, see {@link #setParent(MechanismBase)}.
 */
public class LinearMechanism extends MechanismBase {

    private final double minPosition;
    private final double metersPerUnit;
    private final double massKg;
    /** Which way it moves, with everything at its starting position. */
    private final Translation3d travelDirection;
    private final LinearSimModel simModel;
    private boolean tiltedByParent = false;

    /**
     * @param name Name shown on Shuffleboard and in logs
     * @param motorConfig Applied to every motor; its distance factor sets the position units
     * @param gains PID and feedforward gains, in volts per position unit
     * @param tolerance How close to the setpoint counts as {@link #atSetpoint()}
     * @param minPosition Lowest allowed setpoint, also the starting position
     * @param maxPosition Highest allowed setpoint
     * @param metersPerUnit Meters moved per position unit, 1 if the units are meters; used in simulation and
     *                      {@link #getComponentPose()}
     * @param angleRadians Incline of travel when everything is at its starting position, 0 is horizontal and
     *                     pi/2 is straight up; used in simulation and {@link #getComponentPose()}
     * @param massKg Mass that moves, including everything mounted on it, e.g. a wrist on a telescope; used in
     *               simulation, and for gravity on an arm it's mounted on
     * @param motorType Model of one motor, e.g. {@code DCMotor.getKrakenX60(1)}; used only in simulation
     * @param motors Every motor on the mechanism. The first is the leader.
     */
    public LinearMechanism(String name, MotorConfig motorConfig, PIDFFConfig gains, double tolerance,
                           double minPosition, double maxPosition, double metersPerUnit, double angleRadians,
                           double massKg, DCMotor motorType, NAR_Motor... motors) {
        super(gains, Control.Position, tolerance, minPosition, motorConfig, motors);
        this.minPosition = minPosition;
        this.metersPerUnit = metersPerUnit;
        this.massKg = massKg;
        // Toward the front of the robot (+x), tilted up by the incline
        this.travelDirection = new Translation3d(Math.cos(angleRadians), 0, Math.sin(angleRadians));
        setName(name);
        setInputRange(minPosition, maxPosition);
        setDisableAtSetpoint(false); // keep holding position

        // distanceFactor is position units per encoder rotation, and the encoder may not be on the motor
        final double motorRotationsPerMeter =
            motors[0].getMotorRotationsPerSensorRotation() / (motorConfig.distanceFactor * metersPerUnit);
        simModel = new LinearSimModel(MotorSimModel.gearbox(motorType, motors.length), motorRotationsPerMeter,
            massKg, minPosition * metersPerUnit, maxPosition * metersPerUnit, angleRadians);
        motors[0].enableSim(simModel);
    }

    /**
     * Mounts this on another mechanism, e.g. a telescope on an arm, so its 3D pose moves with it.
     *
     * <p>On something that tilts it, like an arm, how much of its weight pulls along its travel
     * changes with the tilt. kG is then the voltage to hold it pointing straight up, and is
     * multiplied by the sine of its incline on the roboRIO each loop and sent with the setpoint.
     *
     * @param parent What it's mounted on
     */
    @Override
    public void setParent(MechanismBase parent) {
        super.setParent(parent);
        if (parent.tiltsWhatsMountedOnIt()) {
            tiltedByParent = true;
            useRoboRioGravity();
            simModel.setInclineSine(this::getInclineSine);
        }
    }

    /** Which way it moves now, as a unit vector from the robot, including any tilt from what it's mounted on. */
    public Translation3d getTravelDirection() {
        final Rotation3d tilt = getParent() == null ? Rotation3d.kZero : getParent().getComponentPose().getRotation();
        return travelDirection.rotateBy(tilt);
    }

    /** Sine of the incline of its travel now, including any tilt from what it's mounted on. */
    public double getInclineSine() {
        return getTravelDirection().getZ();
    }

    /** How far it has moved out from where it starts, in meters. */
    public double getExtensionMeters() {
        return (getPosition() - minPosition) * metersPerUnit;
    }

    /** Mass moved when it extends, including everything mounted on it. */
    public double getMassKg() {
        return massKg;
    }

    /** kG on the incline, when mounted on something that tilts it. */
    @Override
    protected double calculateFeedforward(double error) {
        // Otherwise the motor controller already applies kG
        return tiltedByParent ? gains.getkG() * getInclineSine() : 0;
    }

    /** How far the moving part is from where it starts, along its travel. */
    @Override
    protected Pose3d getOwnComponentPose() {
        final double meters = getExtensionMeters();
        return new Pose3d(travelDirection.times(meters), Rotation3d.kZero);
    }
}
