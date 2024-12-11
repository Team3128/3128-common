package common.core.subsystems;

import java.util.LinkedList;
import java.util.List;

import org.hamcrest.core.IsEqual;

import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class StateSubsystemBase<S extends Enum<S>> extends SubsystemBase{
    
    protected S state;
    protected final Class<S> enumType;
    protected final List<StandardizedSubsystem> subsystems;
    protected TransitionManager<S> transitionManager;
    private Transition<S> lastScheduledTransition = null;

    
    public StateSubsystemBase(S initialState, Class<S> enumType, TransitionManager<S> transitionManager) {
        state = initialState;
        this.enumType = enumType;
        this.subsystems = new LinkedList<>();
        this.transitionManager = transitionManager;
        initStateTracker();
        registerTransitions();
    }

    public void addSubsystems(StandardizedSubsystem... subsystemsAdditions) {
        for(StandardizedSubsystem subsystem : subsystemsAdditions) {
            subsystems.add(subsystem);
        }
    }

    public void initStateTracker() {
        for(S state : enumType.getEnumConstants()) {
            NAR_Shuffleboard.addData("AmperStates", state.name(), ()-> stateEquals(state), (state.ordinal() % 4), state.ordinal() / 4);
        }
    }

    public abstract void registerTransitions();

    public boolean setState(S nextState) {
        Log.info("COMMANDED", state.name() + " -> " + nextState.name());
        // if not the same state
        if(stateEquals(nextState)) {
            Log.info("COMMANDED", "State already set to " + nextState.name());
            return false;
        }

        Transition<S> transition = transitionManager.getTransition(getState(), nextState);
        // if invalid trnasition
        if(transition == null) {
            Log.info("TRANSITION", "State transition null");
            return false;
        }
        Log.info("TRANSITION", transition.toString());

        if(isTransitioning()) {
            Log.info("TRANSITION", "Already transitioning, procceeding to override");
            lastScheduledTransition.cancel();
        }

        state = nextState;
        lastScheduledTransition = transition;
        lastScheduledTransition.execute();

        if(isTransitioning()) Log.info("State", "Transitioning...");
        return true;
    }

    public Command setStateAsCommand(S nextState) {
        Log.info("State", "Check-1");
        return Commands.runOnce(()-> {Log.info("State", "Check0"); setState(nextState);});//.until(()-> stateEquals(nextState));
    }

    public S getState() {
        return state;
    }

    public boolean stateEquals(S other) {
        return state.name().equals(other.name());
    }

    public boolean isTransitioning() {
        return lastScheduledTransition != null && lastScheduledTransition.isRunning();
    }

    public Command stop() {
        lastScheduledTransition.cancel();
        Command stop = new InstantCommand();
        for(StandardizedSubsystem subsystem : subsystems) {
            stop.andThen(subsystem.stop());
        }
        return stop;
    }

    public Command reset() {
        lastScheduledTransition.cancel();
        Command reset = new InstantCommand();
        for(StandardizedSubsystem subsystem : subsystems) {
            if(subsystem instanceof PositionSubsystemBase) {
                reset.andThen(((PositionSubsystemBase) subsystem).reset());
            }
        }
        return reset;
    }

    public TransitionManager<S> getTransitionManager() {
        return transitionManager;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(getName());
        for(S state : enumType.getEnumConstants()) {
            builder.addBooleanProperty(state.name(), ()-> stateEquals(state), (boolean set)->{
                if(set) setState(state);
            });
        }
    }
}
