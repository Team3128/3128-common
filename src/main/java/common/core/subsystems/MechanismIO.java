package common.core.subsystems;

import common.hardware.motorcontroller.NAR_Motor;

/**
 * Fills a mechanism's inputs box. The mechanism's logic only ever reads the box;
 * which implementation fills it depends on where the code is running.
 */
public interface MechanismIO {

    /** Reads the leader motor and writes its facts into the box. */
    void updateInputs(MechanismIOInputs inputs, NAR_Motor leader, double setpoint);
}
