package common.core.subsystems;

import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Publishes a mechanism's state to NetworkTables under {@code Mechanisms/<name>} every loop.
 * AdvantageScope can graph it live, and DataLogManager records it to the robot's log file
 * along with the rest of NetworkTables.
 */
class MechanismLog {

    private final DoublePublisher setpoint;
    private final DoublePublisher measurement;
    private final DoublePublisher position;
    private final DoublePublisher velocity;
    private final DoublePublisher volts;
    private final BooleanPublisher enabled;
    private final BooleanPublisher atSetpoint;
    // One value per motor, leader first, so a follower that stops pulling current stands out
    private final DoubleArrayPublisher currents;
    private final DoubleArrayPublisher temperatures;

    private final NAR_Motor[] motors;
    private final double[] currentValues;
    private final double[] temperatureValues;

    MechanismLog(String name, NAR_Motor[] motors) {
        final NetworkTable table = NetworkTableInstance.getDefault().getTable("Mechanisms").getSubTable(name);
        setpoint = table.getDoubleTopic("Setpoint").publish();
        measurement = table.getDoubleTopic("Measurement").publish();
        position = table.getDoubleTopic("Position").publish();
        velocity = table.getDoubleTopic("Velocity").publish();
        volts = table.getDoubleTopic("Volts").publish();
        enabled = table.getBooleanTopic("Enabled").publish();
        atSetpoint = table.getBooleanTopic("AtSetpoint").publish();
        currents = table.getDoubleArrayTopic("StatorCurrentAmps").publish();
        temperatures = table.getDoubleArrayTopic("TemperatureCelsius").publish();

        this.motors = motors;
        currentValues = new double[motors.length];
        temperatureValues = new double[motors.length];
    }

    void update(MechanismBase mechanism, double measurementValue) {
        setpoint.set(mechanism.getSetpoint());
        measurement.set(measurementValue);
        position.set(mechanism.getPosition());
        velocity.set(mechanism.getVelocity());
        volts.set(mechanism.getVolts());
        enabled.set(mechanism.isEnabled());
        atSetpoint.set(mechanism.atSetpoint());

        for (int i = 0; i < motors.length; i++) {
            currentValues[i] = motors[i].getStallCurrent();
            temperatureValues[i] = motors[i].getTemperature();
        }
        currents.set(currentValues);
        temperatures.set(temperatureValues);
    }
}
