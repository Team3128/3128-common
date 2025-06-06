package frc.common3128.core.commands;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.function.DoubleSupplier;
import java.util.function.DoubleConsumer;

import frc.common3128.core.controllers.ControllerBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Team 3128's PIDCommand that uses a ControllerBase to control an output.
 * 
 * @since 2024 Crescendo
 * @author Mason Lam
 */
public class NAR_PIDCommand extends Command {
    /** Controller. */
    protected final ControllerBase controller;

    /** Setpoint getter. */
    protected final DoubleSupplier setpointSource;

    public NAR_PIDCommand(
            ControllerBase controller,
            DoubleSupplier measurementSource,
            DoubleSupplier setpointSource,
            DoubleConsumer useOutput,
            Subsystem... requirements) {
        requireNonNullParam(controller, "controller", "NAR_PIDCommand");
        requireNonNullParam(measurementSource, "measurementSource", "NAR_PIDCommand");
        requireNonNullParam(setpointSource, "setpointSource", "NAR_PIDCommand");
        requireNonNullParam(useOutput, "useOutput", "NAR_PIDCommand");
        this.controller = controller;
        this.setpointSource = setpointSource;

        controller.setMeasurementSource(measurementSource);
        controller.addOutput(useOutput);
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        controller.setSetpoint(setpointSource.getAsDouble());
    }

    @Override
    public void execute() {
        controller.useOutput();
    }

    @Override
    public void end(boolean interrupted) {
        controller.reset();
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }
}
