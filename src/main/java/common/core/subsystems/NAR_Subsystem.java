package common.core.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface NAR_Subsystem extends Subsystem{
    
    abstract void reset();
    abstract Command resetCommand();

    abstract void run(double power);
    abstract Command runCommand(double power);

    public void runVolts(double volts);
    public Command runVoltsCommand(double volts);

    public void stop();
    public Command stopCommand();

    public void initShuffleboard();

    public void setNeutralMode(Neutral mode);

    public double getVolts();

    default void runVoltsWidgets(String tab, BooleanSupplier debug, int x, int y) {
        final DoubleSupplier debugVoltage = NAR_Shuffleboard.debug(getName(), "Debug Volts", 0, x, y + 1);
        NAR_Shuffleboard.addCommand(getName(), "Run Volts", either(startEnd(()-> runVolts(debugVoltage.getAsDouble()), ()-> stop()), print("DEBUG NOT ON"), debug), x, y);
        NAR_Shuffleboard.addData(getName(), "Running", ()-> debug.getAsBoolean() && getVolts() > 0, x + 1, y);
        NAR_Shuffleboard.addData(getName(), "Voltage", this::getVolts, x + 1, y + 1);
    }

    default void runWidgets(BooleanSupplier debug, int x, int y) {
        final DoubleSupplier debugPower = NAR_Shuffleboard.debug(getName(), "Debug Power", 0, x, y + 1);
        NAR_Shuffleboard.addCommand(getName(), "Run", either(startEnd(()-> run(debugPower.getAsDouble()), ()-> stop()), print("DEBUG NOT ON"), debug), x, y).withSize(2, 1);
        NAR_Shuffleboard.addData(getName(), "Running", ()-> debug.getAsBoolean() && getVolts() > 0, x + 1, y);
        NAR_Shuffleboard.addData(getName(), "Power", ()-> getVolts() / 12, x + 1, y + 1);
    }

    default void resetWidget(BooleanSupplier debug, int x, int y) {
        NAR_Shuffleboard.addCommand(getName(), "Reset", either(resetCommand(), print("DEBUG NOT ON"), debug), 4, 0);
    }
}
