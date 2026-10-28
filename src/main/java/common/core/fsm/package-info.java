/**
 * State machines: subsystems that are always in one named state and only change state in ways
 * you've allowed.
 *
 * <h2>Why use one</h2>
 * A subsystem like an intake does a few distinct things: sit still, pull game pieces in, spit
 * them out. Writing those as states (NEUTRAL, INTAKE, OUTTAKE) makes the robot's behavior easy to
 * read, easy to show on a dashboard, and hard to get into a bad combination, since a request for
 * a move you haven't allowed is ignored.
 *
 * <h2>The pieces</h2>
 * <ul>
 * <li>{@link common.core.fsm.FSMSubsystemBase}: the subsystem. Extend it, list your states in an
 *     enum, and call {@code setState(...)} to change state.</li>
 * <li>{@link common.core.fsm.Transition}: one allowed move, from one state to another, and the
 *     command that does it.</li>
 * <li>{@link common.core.fsm.TransitionMap}: every allowed move. It has shortcuts for adding many
 *     at once, like "from any state to NEUTRAL".</li>
 * </ul>
 *
 * <p>A state machine can also control other state machines. The robot's Superstructure, for
 * example, has states like SHOOT that set the intake's, hopper's and shooter's states together.
 */
package common.core.fsm;
