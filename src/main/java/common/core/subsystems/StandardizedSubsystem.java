package common.core.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

public interface StandardizedSubsystem {
    
    public Command run(double output);

    public Command runVolts(double volts);

    public Command stop();

    public void initShuffleboard();
}
