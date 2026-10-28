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
 * Runs WPILib's SysId tests on a mechanism and records the results, so the SysId tool can work
 * out its feedforward gains (kS, kV, kA, kG) for you.
 *
 * <p>SysId runs two kinds of test, each forward and backward:
 * <ul>
 * <li><b>Quasistatic</b>: the voltage rises slowly, so the mechanism barely accelerates. This
 *     measures kS and kV.</li>
 * <li><b>Dynamic</b>: the voltage jumps straight to a step, so the mechanism accelerates hard.
 *     This measures kA.</li>
 * </ul>
 * Each test stops after a timeout, 10 seconds unless you give one. For a mechanism with limited
 * travel, like an arm or elevator, pick a timeout short enough that it can't reach the end of its
 * travel, or cancel the test before it does.
 *
 * <p>The results go into the robot's log file. Open it in the SysId tool, which is part of
 * WPILib, to get the gains.
 */
public class NAR_SysIdCommand {


   private SysIdRoutine driveRoutine;
   /** The motor whose position, speed and voltage are recorded. */
   private NAR_Motor logMotor;
   // Reused every loop, to avoid making new objects 50 times a second
   private final MutVoltage appliedVoltage = Volts.mutable(0);
   private final MutDistance position = Meters.mutable(0);
   private final MutLinearVelocity velocity = MetersPerSecond.mutable(0);


   /**
    * Tests that each stop after 10 seconds.
    *
    * @param rampRate Volts per second the quasistatic test's voltage rises by
    * @param stepVoltage Volts the dynamic test jumps to
    * @param setVoltage Drives the mechanism at a voltage, e.g. {@code v -> runVolts(v.in(Volts))}
    * @param sub The subsystem the tests use, so nothing else drives it at the same time
    * @param logMotor The motor to record
    */
   public NAR_SysIdCommand(double rampRate, double stepVoltage, Consumer<Voltage> setVoltage, Subsystem sub, NAR_Motor logMotor) {
       this(rampRate, stepVoltage, 10, setVoltage, sub, logMotor);
   }

   /**
    * @param rampRate Volts per second the quasistatic test's voltage rises by
    * @param stepVoltage Volts the dynamic test jumps to
    * @param timeoutSeconds How long each test runs before it stops
    * @param setVoltage Drives the mechanism at a voltage, e.g. {@code v -> runVolts(v.in(Volts))}
    * @param sub The subsystem the tests use, so nothing else drives it at the same time
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
    * Records the motor's position, speed and voltage, every loop while a test runs. They're
    * labeled meters, but they're in the mechanism's own units; SysId's gains come out in those.
    */
   public void logMotors(SysIdRoutineLog log){
       log.motor("mod0-motor")
           .linearPosition(position.mut_replace(logMotor.getPosition(), Meters))
           .linearVelocity(velocity.mut_replace(logMotor.getVelocity(), MetersPerSecond))
           .voltage(appliedVoltage.mut_replace(logMotor.getAppliedOutput() * 12, Volts));
   }


   /** All four tests in a row, with a second's pause between them. */
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
