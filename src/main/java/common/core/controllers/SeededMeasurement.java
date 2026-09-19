package common.core.controllers;

import java.util.function.DoubleSupplier;

import common.hardware.motorcontroller.NAR_Motor;

/**
 * Composes an absolute encoder (e.g. a CANcoder) with a motor's relative encoder: the
 * absolute source is read once, to seed the motor's position, and never read again. The
 * motor's own encoder is faster-updating and more precise than polling an absolute sensor
 * every loop, so it alone should drive the control loop after seeding.
 *
 * <p>Equivalent to what {@code SwerveModule.resetToAbsolute()} does for the angle motor,
 * generalized so any mechanism with an absolute encoder can reuse it.
 */
public final class SeededMeasurement {

    private SeededMeasurement() {}

    /**
     * Seeds {@code motor}'s position from {@code absoluteSource} once, then returns a
     * supplier backed entirely by the motor's own (relative) encoder.
     *
     * @param motor The motor whose relative encoder will drive the control loop.
     * @param absoluteSource The absolute encoder reading to seed from, in the same units
     *                        as {@code motor.getPosition()}.
     */
    public static DoubleSupplier position(NAR_Motor motor, DoubleSupplier absoluteSource) {
        motor.resetPosition(absoluteSource.getAsDouble());
        return motor::getPosition;
    }
}
