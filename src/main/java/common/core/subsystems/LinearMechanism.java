package common.core.subsystems;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.sim.LinearSimModel;
import common.hardware.motorcontroller.sim.MotorSimModel;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Moves in a straight line to a position and holds it, at any incline:
 * sliders (0 degrees), elevators (90 degrees), or anything in between.
 *
 * <p>PID and feedforward run on the motor controller. Feedforward is kS plus a constant kG
 * from {@code gains}; leave kG at 0 for a horizontal slider. The mechanism is assumed to be at {@code minPosition}
 * when the robot turns on.
 */
public class LinearMechanism extends MechanismBase {

    /**
     * @param name Name shown on Shuffleboard and in logs
     * @param motorConfig Applied to every motor; its distance factor sets the position units
     * @param gains PID and feedforward gains, in volts per position unit
     * @param tolerance How close to the setpoint counts as {@link #atSetpoint()}
     * @param minPosition Lowest allowed setpoint, also the starting position
     * @param maxPosition Highest allowed setpoint
     * @param metersPerUnit Meters moved per position unit, 1 if the units are meters; used only in simulation
     * @param angleRadians Incline of travel, 0 is horizontal and pi/2 is straight up; used only in simulation
     * @param massKg Mass that moves; used only in simulation
     * @param motorType Model of one motor, e.g. {@code DCMotor.getKrakenX60(1)}; used only in simulation
     * @param motors Every motor on the mechanism. The first is the leader.
     */
    public LinearMechanism(String name, MotorConfig motorConfig, PIDFFConfig gains, double tolerance,
                           double minPosition, double maxPosition, double metersPerUnit, double angleRadians,
                           double massKg, DCMotor motorType, NAR_Motor... motors) {
        super(gains, Control.Position, tolerance, minPosition, motorConfig, motors);
        setName(name);
        setInputRange(minPosition, maxPosition);
        setDisableAtSetpoint(false); // keep holding position

        // distanceFactor is position units per motor rotation
        final double motorRotationsPerMeter = 1 / (motorConfig.distanceFactor * metersPerUnit);
        motors[0].enableSim(new LinearSimModel(MotorSimModel.gearbox(motorType, motors.length), motorRotationsPerMeter,
            massKg, minPosition * metersPerUnit, maxPosition * metersPerUnit, angleRadians));
    }
}
