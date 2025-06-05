package common.hardware.motorcontroller;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class NAR_MotorIOAutoLogged extends NAR_Motor.NAR_MotorIO implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("InputPower", inputPower);
    table.put("AppliedOutput", appliedOutput);
    table.put("StallCurrent", stallCurrent);
    table.put("Position", position);
    table.put("Velocity", velocity);
  }

  @Override
  public void fromLog(LogTable table) {
    inputPower = table.get("InputPower", inputPower);
    appliedOutput = table.get("AppliedOutput", appliedOutput);
    stallCurrent = table.get("StallCurrent", stallCurrent);
    position = table.get("Position", position);
    velocity = table.get("Velocity", velocity);
  }

  public NAR_MotorIOAutoLogged clone() {
    NAR_MotorIOAutoLogged copy = new NAR_MotorIOAutoLogged();
    copy.inputPower = this.inputPower;
    copy.appliedOutput = this.appliedOutput;
    copy.stallCurrent = this.stallCurrent;
    copy.position = this.position;
    copy.velocity = this.velocity;
    return copy;
  }
}
