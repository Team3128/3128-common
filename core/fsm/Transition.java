package frc.common3128.core.fsm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Transition<T extends Enum<T>> {
    
    private final T outgoingState;
    private final T incomingState;
    private final Command command;

    public Transition(T outgoingState, T incomingState, Command command){
        this.outgoingState = outgoingState;
        this.incomingState = incomingState;
        this.command = command;
    }

    public Transition(T outgoingState, T incomingState, Runnable action){
        this(outgoingState, incomingState, Commands.runOnce(action));
    }

    public Transition(T outgoingState, T incomingState){
        this(outgoingState, incomingState, Commands.none());
    }

    public T getOutgoingState(){
        return outgoingState;
    }

    public T getIncomingState(){
        return incomingState;
    }

    public Command getCommand(){
        return command;
    }

    public boolean isTransition(T currentState, T nextState){
        return currentState.equals(outgoingState) && nextState.equals(incomingState);
    }

    public boolean equals(Transition<T> other) {
        return isTransition(other.getOutgoingState(), other.getIncomingState());
    }

    public String toString() {
        return outgoingState.name() + " -> " + incomingState.name();
    }

    public void cancel() {
        command.cancel();
    }

    public void execute() {
        command.schedule();
    }

    public boolean isScheduled() {
        return command.isScheduled();
    }

    public boolean isFinished() {
        return command.isFinished();
    }
}