package common.core.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import common.core.controllers.ControllerBase;
import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class TalonPositionSubsystemBase extends NAR_TalonPIDSubsystem implements NAR_Subsystem {
    protected NAR_TalonFX narTalon;
    protected MotionMagicVoltage m_request;

    public TalonPositionSubsystemBase(PIDFFConfig config, NAR_TalonFX motor, double cruiseVelocity, double acceleration) {
        super(config, motor, cruiseVelocity, acceleration);

        this.narTalon = motor;
        this.m_request = new MotionMagicVoltage(0);

    }

    /**
     * Sets controller setpoint and enables controller.
     * 
     * @param setpoint Setpoint the pivot goes to.
     * @return Command setting pivot setpoint.
     */
    public Command pidTo(double setpoint) {
        return runOnce(() -> motor.setControl(m_request.withPosition(setpoint)));
    }

    /**
     * Sets controller setpoint and enables controller.
     * 
     * @param setpoint Setpoint the pivot goes to.
     * @return Command setting pivot setpoint.
     */
    public Command pidTo(DoubleSupplier setpoint) {
        return runOnce(()-> pidTo(setpoint.getAsDouble()));
    }

    /**
     * Sets voltage to motors.
     * 
     * @param volts The voltage to set the motors to.
     */
    public void runVolts(double volts) {
        System.out.println("Running Volts at " + volts);
        narTalon.setVolts(volts);
    }

    /**
     * Command resetting position to controller position minimum.
     * 
     * @return Command resetting the position to the controller position minimum.
     */
    public Command resetCommand() {
        return Commands.runOnce(() -> reset());
    }

    /**
     * Command setting voltage to motors.
     * 
     * @param volts The voltage to set the motors to.
     * @return Command setting the voltage to the motors.
     */
    public Command runVoltsCommand(double volts) {
        return Commands.runOnce(() -> runVolts(volts));
    }

    public double getVolts() {
        return narTalon.getAppliedOutput() * 12;
    }

    /**
     * Set the neutral mode for all motors in the mechanism.
     * 
     * @param mode The neutral mode to set to.
     */
    public void setNeutralMode(Neutral mode) {
        narTalon.setNeutralMode(mode);
    }

    @Override
    public void initShuffleboard() {
        super.initShuffleboard();
        NAR_Shuffleboard.addData(getName(), "Current", ()-> narTalon.getStallCurrent(), 5, 0);
        NAR_Shuffleboard.addCommand(getName(), "Enable", Commands.either(startEnd(()-> startPID(setpoint.getAsDouble()), ()-> disable()), Commands.print("DEBUG NOT ON"), debug), 4, 0);
        FFWidgets( controller,1,0);
        runVoltsWidgets("running", debug, 1, 0);
        resetWidget( debug, 1, 0);
        runWidgets(debug,6,3);

    }

    public void FFWidgets(ControllerBase controller, int x, int y) {
        controller.getConfig().setkS(NAR_Shuffleboard.debug(getName(), "kS", controller.getConfig().getkS(), x, y+2));
        controller.getConfig().setkV(NAR_Shuffleboard.debug(getName(), "kV", controller.getConfig().getkV(), x + 1, y+2));
        controller.getConfig().setkA(NAR_Shuffleboard.debug(getName(), "kA", controller.getConfig().getkA(), x + 1, y + 3));
        controller.getConfig().setkG(NAR_Shuffleboard.debug(getName(), "kG", controller.getConfig().getkG(), x, y + 3));
    }
}
