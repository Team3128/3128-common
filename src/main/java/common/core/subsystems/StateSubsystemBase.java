package common.core.subsystems;

import java.util.LinkedList;
import java.util.List;

import org.hamcrest.core.IsEqual;

import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class StateSubsystemBase<S extends Enum<S>> extends SubsystemBase{
    
    private S state;
    private final Class<S> enumType;
    private final List<StandardizedSubsystem> subsystems;
    private TransitionManager<S> transitionManager;
    private Transition<S> requTransition = null;
    private Transition<S> currTransition = null;

    
    public StateSubsystemBase(S initialState, Class<S> enumType) {
        state = initialState;
        this.enumType = enumType;
        this.subsystems = new LinkedList<>();
        transitionManager = new TransitionManager<>(enumType);
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
        Transition<S> transition = transitionManager.getTransition(getState(), nextState);

        // if not the same state
        if(!stateEquals(nextState)) requTransition = transition;
        else return false;

        // if invalid trnasition
        if(transition == null) return false;

        // if not transitioning
        if(!isTransitioning()) {
            currTransition = transition;
            currTransition.execute();
            return true;
        }

        return false;
    }

    public Command setStateAsCommand(S nextState) {
        return Commands.runOnce(()-> setState(nextState)).until(()-> stateEquals(nextState));
    }

    public S getState() {
        return state;
    }

    public boolean stateEquals(S other) {
        return state.name().equals(other.name());
    }

    public Transition<S> getRequTransition() {
        return requTransition;
    }

    public Transition<S> getCurrTransition() {
        return currTransition;
    }

    public boolean isTransitioning() {
        return currTransition != null;
    }

    public Command stop() {
        currTransition.cancel();
        Command stop = new InstantCommand();
        for(StandardizedSubsystem subsystem : subsystems) {
            stop.andThen(subsystem.stop());
        }
        return stop;
    }

    public Command reset() {
        currTransition.cancel();
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
    public void periodic() {
        if (isTransitioning() && currTransition.isFinished()) {
            currTransition = null;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        for(S state : enumType.getEnumConstants()) {
            builder.addBooleanProperty(state.name(), ()-> stateEquals(state), (boolean set)->{
                if(set) setState(state);
            });
        }
    }
}
