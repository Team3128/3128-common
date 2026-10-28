package common.core.fsm;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * One allowed move in a state machine: from one state to another, and the command that does it.
 * For example, NEUTRAL to INTAKE with a command that extends the intake and starts its roller.
 *
 * <p>You rarely make these yourself; the {@link TransitionMap} add methods build them for you.
 *
 * @param <T> The enum listing the states
 */
public class Transition<T extends Enum<T>> {

    /** The state it leaves. */
    private final T outgoingState;
    /** The state it enters. */
    private final T incomingState;
    private final Command command;

    /**
     * @param outgoingState The state it leaves
     * @param incomingState The state it enters
     * @param command What to run to get there
     */
    public Transition(T outgoingState, T incomingState, Command command){
        this.outgoingState = outgoingState;
        this.incomingState = incomingState;
        this.command = command;
    }

    /** A transition that runs {@code action} once, e.g. {@code () -> roller.run(0.5)}. */
    public Transition(T outgoingState, T incomingState, Runnable action){
        this(outgoingState, incomingState, Commands.runOnce(action));
    }

    /** A transition that's allowed but does nothing; only the state changes. */
    public Transition(T outgoingState, T incomingState){
        this(outgoingState, incomingState, Commands.none());
    }

    /** The state it leaves. */
    public T getOutgoingState(){
        return outgoingState;
    }

    /** The state it enters. */
    public T getIncomingState(){
        return incomingState;
    }

    public Command getCommand(){
        return command;
    }

    /** Whether this is the transition from {@code currentState} to {@code nextState}. */
    public boolean isTransition(T currentState, T nextState){
        return currentState.equals(outgoingState) && nextState.equals(incomingState);
    }

    /** Transitions are equal when they go between the same two states, whatever their commands. */
    @Override
    public boolean equals(Object other) {
        return other instanceof Transition<?> transition
            && outgoingState == transition.outgoingState
            && incomingState == transition.incomingState;
    }

    @Override
    public int hashCode() {
        return Objects.hash(outgoingState, incomingState);
    }

    /** e.g. {@code "NEUTRAL -> INTAKE"} */
    public String toString() {
        return outgoingState.name() + " -> " + incomingState.name();
    }

    /** Stops its command if it's running. */
    public void cancel() {
        command.cancel();
    }

    /** Starts its command. */
    public void execute() {
        CommandScheduler.getInstance().schedule(command);
    }

    /** Whether its command is running. */
    public boolean isScheduled() {
        return command.isScheduled();
    }

    /** Asks its command whether it's done, using {@link Command#isFinished()}. */
    public boolean isFinished() {
        return command.isFinished();
    }
}
