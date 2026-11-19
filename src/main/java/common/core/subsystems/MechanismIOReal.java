package common.core.subsystems;

import common.hardware.motorcontroller.NAR_Motor;

/** Fills the inputs box from the real leader motor. */
public class MechanismIOReal implements MechanismIO {

    @Override
    public void updateInputs(MechanismIOInputs inputs, NAR_Motor leader, double setpoint) {
        inputs.position = leader.getPosition();
        inputs.velocity = leader.getVelocity();
        inputs.volts = leader.getAppliedOutput() * MechanismBase.MAX_VOLTS;
        inputs.setpoint = setpoint;
    }
}
