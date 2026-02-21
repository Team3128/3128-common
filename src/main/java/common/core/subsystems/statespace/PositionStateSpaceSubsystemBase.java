package common.core.subsystems.statespace;

import common.core.controllers.PIDFFConfig;
import common.core.subsystems.NAR_Subsystem;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.Command;

public class PositionStateSpaceSubsystemBase extends NAR_StateSpaceSubsystemBase<N2> implements NAR_Subsystem{

    public PositionStateSpaceSubsystemBase(PIDFFConfig config, Vector<N2> stateSTD, Vector<N2> measurementSTD, Vector<N2> qelms, Vector<N1> relms, NAR_Motor motor) {
        super(LinearSystemId.identifyPositionSystem(config.getkV(), config.getkA()), stateSTD, measurementSTD, qelms, relms, motor);
    }

    @Override
    public void setStateSetpoint(double setpoint) {
        loop.setNextR(VecBuilder.fill(setpoint, 0));
    }

    @Override
    public Vector<N2> getMeasurement() {
        return VecBuilder.fill(motor.getPosition(), motor.getVelocity());
    }

    @Override
    public void reset() {
        motor.resetPosition(0);
    }

    @Override
    public Command resetCommand() {
        return runOnce(() -> reset());
        // throw new UnsupportedOperationException("Unimplemented method 'resetCommand'");
    }

    @Override
    public void run(double power) {
        motor.set(power);
        // throw new UnsupportedOperationException("Unimplemented method 'run'");
    }

    @Override
    public Command runCommand(double power) {
        return runOnce(() -> run(power));
    }

    @Override
    public void runVolts(double volts) {
        motor.set(volts/12);
    }

    @Override
    public Command runVoltsCommand(double volts) {
        return runOnce(() -> runVolts(volts));
    }

    @Override
    public void stop() {
        motor.set(0);
    }

    @Override
    public Command stopCommand() {
        return runOnce(()-> stop());
    }

    @Override
    public void initShuffleboard() {
        NAR_Shuffleboard.addData(getName(), "Position", getMeasurement().get(0),0,0);
        NAR_Shuffleboard.addData(getName(), "Velocity", getMeasurement().get(1),0,1);

    }

    @Override
    public void setNeutralMode(Neutral mode) {
        motor.setNeutralMode(mode);
    }

    @Override
    public double getVolts() {
        return motor.getAppliedOutput() * 12;
    }
}
