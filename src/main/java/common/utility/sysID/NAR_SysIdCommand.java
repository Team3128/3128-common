package common.utility.sysID;

import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.utility.Log;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

public class NAR_SysIdCommand {

    private SysIdRoutine driveRoutine;
    private NAR_Motor logMotor;
    private final MutVoltage appliedVoltage = Volts.mutable(0);
    private final MutDistance position = Meters.mutable(0);
    private final MutLinearVelocity velocity = MetersPerSecond.mutable(0);

    
    public NAR_SysIdCommand(double rampRate, double stepVoltage, Consumer<Voltage> setVoltage, Subsystem sub, NAR_Motor logMotor) {
        driveRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(rampRate).per(Second), Volts.of(stepVoltage), null),
            new SysIdRoutine.Mechanism(setVoltage, this::logMotors, sub));
        this.logMotor = logMotor;
    }

    public void logMotors(SysIdRoutineLog log){
        log.motor("mod0-motor")
            .linearPosition(position.mut_replace(logMotor.getPosition(), Meters))
            .linearVelocity(velocity.mut_replace(logMotor.getVelocity(), MetersPerSecond))
            .voltage(appliedVoltage.mut_replace(logMotor.getAppliedOutput() * 12, Volts));
        Log.info("Debug Position", logMotor.getPosition());
        Log.info("Debug Velocity", logMotor.getVelocity());
        Log.info("Debug Voltage", logMotor.getAppliedOutput() * 12);
    }

    public Command runSysId(){
        return sequence(
            driveRoutine.quasistatic(SysIdRoutine.Direction.kForward),
            waitSeconds(1),
            driveRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
            waitSeconds(1),
            driveRoutine.dynamic(SysIdRoutine.Direction.kForward),
            waitSeconds(1),
            driveRoutine.dynamic(SysIdRoutine.Direction.kReverse)
        );
    }
}
