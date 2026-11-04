package common.utility.sysid;


import common.hardware.motorcontroller.NAR_Motor;
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


/**
 * Runs WPILib's SysId tests on a mechanism and records the results, so that the SysId tool can
 * calculate its feedforward gains (kS, kV, kA, kG).
 *
 * <p>SysId runs two kinds of test, each in both directions:
 * <ul>
 * <li><b>Quasistatic</b>: the voltage rises slowly, so the mechanism barely accelerates. This
 *     measures kS and kV.</li>
 * <li><b>Dynamic</b>: the voltage steps immediately to a set value, so the mechanism accelerates quickly.
 *     This measures kA.</li>
 * </ul>
 * Each test stops after a timeout, 10 seconds by default. For a mechanism with limited
 * travel, such as an arm or elevator, choose a timeout short enough that it cannot reach the end of its
 * travel, or cancel the test before it does.
 *
 * <p>The results are stored in the robot's log file. Open that file in the SysId tool, which is part of
 * WPILib, to get the gains.
 */
public class NAR_SysIdCommand {


   private SysIdRoutine driveRoutine;
   /** The motor whose position, speed and voltage are recorded. */
   private NAR_Motor logMotor;
   /** Reused every loop, as are the next two fields, to avoid allocating objects 50 times a second. */
   private final MutVoltage appliedVoltage = Volts.mutable(0);
   private final MutDistance position = Meters.mutable(0);
   private final MutLinearVelocity velocity = MetersPerSecond.mutable(0);


   /**
    * Creates tests that each stop after 10 seconds.
    *
    * @param rampRate The rate at which the quasistatic test's voltage rises, in volts per second
    * @param stepVoltage The voltage of the dynamic test's step
    * @param setVoltage Drives the mechanism at a voltage, for example, {@code v -> runVolts(v.in(Volts))}
    * @param sub The subsystem the tests require, so that nothing else drives it at the same time
    * @param logMotor The motor to record
    */
   public NAR_SysIdCommand(double rampRate, double stepVoltage, Consumer<Voltage> setVoltage, Subsystem sub, NAR_Motor logMotor) {
       this(rampRate, stepVoltage, 10, setVoltage, sub, logMotor);
   }

   /**
    * @param rampRate The rate at which the quasistatic test's voltage rises, in volts per second
    * @param stepVoltage The voltage of the dynamic test's step
    * @param timeoutSeconds The duration of each test before it stops
    * @param setVoltage Drives the mechanism at a voltage, for example, {@code v -> runVolts(v.in(Volts))}
    * @param sub The subsystem the tests require, so that nothing else drives it at the same time
    * @param logMotor The motor to record
    */
   public NAR_SysIdCommand(double rampRate, double stepVoltage, double timeoutSeconds, Consumer<Voltage> setVoltage,
                           Subsystem sub, NAR_Motor logMotor) {
       driveRoutine = new SysIdRoutine(
           new SysIdRoutine.Config(Volts.of(rampRate).per(Second), Volts.of(stepVoltage), Seconds.of(timeoutSeconds)),
           new SysIdRoutine.Mechanism(setVoltage, this::logMotors, sub));
       this.logMotor = logMotor;
   }


   /**
    * Records the motor's position, speed and voltage every loop while a test runs. The values are
    * labeled as meters, but they are in the mechanism's own units, and SysId's gains use those units.
    */
   public void logMotors(SysIdRoutineLog log){
       log.motor("mod0-motor")
           .linearPosition(position.mut_replace(logMotor.getPosition(), Meters))
           .linearVelocity(velocity.mut_replace(logMotor.getVelocity(), MetersPerSecond))
           .voltage(appliedVoltage.mut_replace(logMotor.getAppliedOutput() * 12, Volts));
   }


   /** Returns a command that runs all four tests in sequence, with a one-second pause between them. */
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
