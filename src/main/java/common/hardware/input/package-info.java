/**
 * Controllers the drivers hold: {@link common.hardware.input.NAR_XboxController},
 * {@link common.hardware.input.NAR_Joystick} and the team's
 * {@link common.hardware.input.NAR_ButtonBoard}.
 *
 * <p>Each one turns its buttons into {@code Trigger}s you bind commands to, gives its sticks a
 * small deadband so they read 0 when let go, and makes up and forward positive.
 *
 * <p>The number you create one with is its port in the Driver Station's USB tab. In simulation,
 * drag a controller onto that port in the sim window's Joysticks panel.
 */
package common.hardware.input;
