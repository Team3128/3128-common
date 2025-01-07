package common.core.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface NAR_Subsystem extends Subsystem{

    /**
     * Configure motor settings.
     */
    abstract void configMotors();
    
    abstract Command reset();

    abstract Command run();

    public Command runVolts();

    public Command stop();

    public void initShuffleboard();

    public double getCurrent();
}
