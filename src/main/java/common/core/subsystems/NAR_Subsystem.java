package common.core.subsystems;

import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.wpilibj2.command.Command;
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
}
