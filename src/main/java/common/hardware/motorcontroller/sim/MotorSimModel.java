package common.hardware.motorcontroller.sim;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Physics for whatever a motor is driving, used only in simulation.
 * Attach one to a motor with {@code NAR_Motor.enableSim(model)}.
 *
 * <p>Everything is measured at the motor (rotor) shaft, because that is what the
 * real motor controller's encoder reports. Each model starts at 0 rotations, like a
 * real motor does when the robot turns on.
 */
public interface MotorSimModel {

    /** The motor(s) driving the mechanism, e.g. {@code DCMotor.getKrakenX44(1)}. */
    DCMotor getMotor();

    /** Advances the physics by {@code dtSeconds} with {@code motorVolts} applied to the motor. */
    void update(double motorVolts, double dtSeconds);

    /** Rotor position in rotations. */
    double getRotorRotations();

    /** Rotor velocity in rotations per second. */
    double getRotorRPS();

    /** Current drawn by the motor(s) in amps. */
    double getCurrentAmps();
}
