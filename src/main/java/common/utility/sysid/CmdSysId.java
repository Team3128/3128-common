package common.utility.sysid;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Team 3128's command to identify feed forward constants.
 * @since 2024 Crescendo
 * @author Teja Yaramada, Audrey Zheng, William Yuan
 */
public class CmdSysId extends Command {

    private final FFCharacterization data;
    private final Consumer<Double> voltageConsumer;
    private final Supplier<Double> velocitySupplier;
    private final Supplier<Double> positionSupplier;

    private final double startDelaySecs;
    private final double rampRateVoltsPerSec;
    private final double targetPosition;

    private final Timer timer = new Timer();

    /**
     * Creates a new command to identify feed forward constants of the subsystem.
     * @param name Name of the test.
     * @param voltageConsumer Motors of the subsystem.
     * @param velocitySupplier Velocity of the subsystem.
     * @param startDelaySecs Delay before the command starts.
     * @param rampRateVoltsPerSec Rate at which voltage increases.
     * @param subsystems Subsystems used.
     */
    public CmdSysId(
        String name,
        Consumer<Double> voltageConsumer,
        Supplier<Double> velocitySupplier,
        Supplier<Double> positionSupplier,
        double startDelaySecs,
        double rampRateVoltsPerSec,
        double targetPosition,
        Subsystem... subsystems
        ) {
        this.data = new FFCharacterization(name);
        this.voltageConsumer = voltageConsumer;
        this.velocitySupplier = velocitySupplier;
        this.positionSupplier = positionSupplier;
        this.startDelaySecs = startDelaySecs;
        this.rampRateVoltsPerSec = rampRateVoltsPerSec;
        this.targetPosition = targetPosition;
        
        addRequirements(subsystems);
    }

    /**
     * Creates a new command to identify feed forward constants of the subsystem.
     * @param name Name of the test.
     * @param voltageConsumer Motors of the subsystem.
     * @param velocitySupplier Velocity of the subsystem.
     * @param subsystems Subsystems used.
     */
    public CmdSysId(
        String name,
        Consumer<Double> voltageConsumer,
        Supplier<Double> velocitySupplier,
        Supplier<Double> positionSupplier,
        double targetPosition,
        Subsystem... subsystems
        ) {
        this(name, voltageConsumer, velocitySupplier, positionSupplier, 2.0, 1.0, targetPosition, subsystems);
    }

    @Override
    public void initialize() {
        data.clear();
        setVoltage(0.0);
        timer.restart();
    }

    @Override
    public void execute() {
        if (timer.get() < startDelaySecs) return;

        final double voltage = (timer.get() - startDelaySecs) * rampRateVoltsPerSec;

        setVoltage(voltage);
        updateData(voltage);
    }

    @Override
    public void end(boolean interrupted) {
        setVoltage(0.0);
        timer.stop();
        data.print();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(positionSupplier.get()) >= targetPosition;
    }

    /**
     * Sets voltage of subsystem motors.
     * @param voltage Voltage to be applied to subsystem
     */
    private void setVoltage(double voltage) {
        voltageConsumer.accept(voltage);
    }

    /**
     * Adds current voltage applied and subsystem velocity data.
     * @param voltage Current voltage applied to subsystem.
     */
    private void updateData(double voltage) {
        data.add(timer.get() - startDelaySecs, velocitySupplier.get(), voltage);
    }
}