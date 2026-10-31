package common.hardware.motorcontroller.sim;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * A physics model of what a motor drives, used only in simulation.
 * Attach one to a motor with {@code NAR_Motor.enableSim(model)}.
 *
 * <p>All values are measured at the motor (rotor) shaft, because that is what the
 * real motor controller's encoder reports. Each model starts at 0 rotations, as a
 * real motor does when the robot turns on.
 */
public interface MotorSimModel {

    /** Returns the motor(s) driving the mechanism, for example, {@code DCMotor.getKrakenX44(1)}. */
    DCMotor getMotor();

    /** Advances the physics by {@code dtSeconds} with {@code motorVolts} applied to the motor. */
    void update(double motorVolts, double dtSeconds);

    /** Returns the rotor position in rotations. */
    double getRotorRotations();

    /** Returns the rotor velocity in rotations per second. */
    double getRotorRPS();

    /** Returns the current drawn by the motor(s) in amps. */
    double getCurrentAmps();

    /**
     * Combines several identical motors driving one mechanism.
     * @param oneMotor The model of a single motor, for example, {@code DCMotor.getKrakenX44(1)}
     * @param count The number of motors driving the mechanism
     * @return A model of the motors combined
     */
    static DCMotor gearbox(DCMotor oneMotor, int count) {
        return new DCMotor(oneMotor.nominalVoltageVolts, oneMotor.stallTorqueNewtonMeters, oneMotor.stallCurrentAmps,
            oneMotor.freeCurrentAmps, oneMotor.freeSpeedRadPerSec, count);
    }
}
