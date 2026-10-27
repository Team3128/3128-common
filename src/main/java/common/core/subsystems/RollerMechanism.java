package common.core.subsystems;

import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.sim.MotorSimModel;
import common.hardware.motorcontroller.sim.RollerSimModel;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Spins at a set power or voltage with no feedback: intakes, hoppers, kickers.
 * Use {@link #run(double)} or {@link #runVolts(double)}.
 */
public class RollerMechanism extends MechanismBase {

    /**
     * @param name Name shown on Shuffleboard and in logs
     * @param motorConfig Applied to every motor
     * @param motorType Model of one motor, e.g. {@code DCMotor.getKrakenX60(1)}; used only in simulation
     * @param gearing Motor rotations per roller rotation; used only in simulation
     * @param moiKgMetersSq Roller inertia, see {@link RollerSimModel#solidCylinderMoi} and
     *                      {@link RollerSimModel#hollowCylinderMoi}; used only in simulation
     * @param motors Every motor on the roller. All of them get the same output.
     */
    public RollerMechanism(String name, MotorConfig motorConfig, DCMotor motorType, double gearing,
                           double moiKgMetersSq, NAR_Motor... motors) {
        super(motorConfig, motors);
        setName(name);
        motors[0].enableSim(new RollerSimModel(MotorSimModel.gearbox(motorType, motors.length), gearing, moiKgMetersSq));
    }
}
