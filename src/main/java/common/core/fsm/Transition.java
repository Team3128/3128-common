package common.core.fsm;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * One allowed move in a state machine: from one state to another, with the command that performs
 * it. For example, NEUTRAL to INTAKE with a command that extends the intake and starts its roller.
 *
 * <p>Instances are rarely created directly; the {@link TransitionMap} add methods build them.
 *
 * @param <T> The enum listing the states
 */
public class Transition<T extends Enum<T>> {

    /** The state being left. */
    private final T outgoingState;
    /** The state being entered. */
    private final T incomingState;
    private final Command command;

    /**
     * @param outgoingState The state being left
     * @param incomingState The state being entered
     * @param command The command that performs the transition
     */
    public Transition(T outgoingState, T incomingState, Command command){
        this.outgoingState = outgoingState;
        this.incomingState = incomingState;
        this.command = command;
    }

    /** Creates a transition that runs {@code action} once, for example, {@code () -> roller.run(0.5)}. */
    public Transition(T outgoingState, T incomingState, Runnable action){
        this(outgoingState, incomingState, Commands.runOnce(action));
    }

    /** Creates a transition that is allowed but performs no action; only the state changes. */
    public Transition(T outgoingState, T incomingState){
        this(outgoingState, incomingState, Commands.none());
    }

    /** Returns the state being left. */
    public T getOutgoingState(){
        return outgoingState;
    }

    /** Returns the state being entered. */
    public T getIncomingState(){
        return incomingState;
    }

    public Command getCommand(){
        return command;
    }

    /** Returns whether this is the transition from {@code currentState} to {@code nextState}. */
    public boolean isTransition(T currentState, T nextState){
        return currentState.equals(outgoingState) && nextState.equals(incomingState);
    }

    /** Transitions are equal when they connect the same two states, regardless of their commands. */
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

    /** Returns a description such as {@code "NEUTRAL -> INTAKE"}. */
    public String toString() {
        return outgoingState.name() + " -> " + incomingState.name();
    }

    /** Cancels the command if it is running. */
    public void cancel() {
        command.cancel();
    }

    /** Starts the command. */
    public void execute() {
        CommandScheduler.getInstance().schedule(command);
    }

    /** Returns whether the command is running. */
    public boolean isScheduled() {
        return command.isScheduled();
    }

    /** Returns whether the command has finished, using {@link Command#isFinished()}. */
    public boolean isFinished() {
        return command.isFinished();
    }
}
