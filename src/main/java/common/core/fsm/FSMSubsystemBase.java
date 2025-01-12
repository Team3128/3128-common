package common.core.fsm;

import java.util.LinkedList;
import java.util.List;

import common.core.subsystems.NAR_Subsystem;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class FSMSubsystemBase<S extends Enum<S>> extends SubsystemBase {
    
    private Transition<S> currentTransition;
    private S currentState;
    private Transition<S> requestTransition;
    private S previousState;

    private final TransitionMap<S> transitionMap;
    private final Class<S> enumType;

    protected static LinkedList<NAR_Subsystem> subsystems;

    public FSMSubsystemBase(Class<S> enumType, TransitionMap<S> transitionMap) {
        this.enumType = enumType;
        this.transitionMap = transitionMap;
        registerTransitions();
        initStateTracker();
    }

    public FSMSubsystemBase(Class<S> enumType, TransitionMap<S> transitionMap, S initalState) {
        this(enumType, transitionMap);
        this.currentState = initalState;
    }

    public void initStateTracker() {
        for(S state : enumType.getEnumConstants()) {
            NAR_Shuffleboard.addData(this.getName(), state.name(), ()-> stateEquals(state), (state.ordinal() % 4), state.ordinal() / 4);
        }
    }

    public void setState(S nextState) {
        Transition<S> transition = transitionMap.getTransition(getState(), nextState);

        // if not the same state
        if(!stateEquals(nextState)) requestTransition = transition;
        else return;

        // if invalid trnasition
        if(transition == null) return;

        // if not transitioning
        if(isTransitioning()) {
            currentTransition.cancel();
        }

        currentTransition = transition;
        currentTransition.getCommand().schedule();
    }

    public Command setStateCommand(S nextState) {
        return Commands.runOnce(()-> setState(nextState));
    }

    public boolean stateEquals(S state) {
        if(state == null || currentState == null) return false;
        return currentState.equals(state);
    }

    public S getState() {
        return currentState;
    }

    public S getPreviousState() {
        return previousState;
    }

    public Transition<S> getCurrentTransition() {
        return currentTransition;
    }

    public Transition<S> getRequestTransition()  {
        return requestTransition;
    }

    public boolean isTransitioning() {
        return currentTransition.isFinished();
    }

    /**
     * Update the adjacency map to include the data for the given transition Note: This WILL NOT
     * overwrite an existing Transition
     *
     * @param transition The transition to add to the graph
     */
    public void addTransition(Transition<S> transition) {
        transitionMap.addTransition(transition);
    }

    public void addTransition(S start, S end, Command command) {
        addTransition(new Transition<S>(start, end, command));
    }

    public abstract void registerTransitions();

    public void addSubsystem(NAR_Subsystem... subsystem) {
        for(NAR_Subsystem sub : subsystem) {
            subsystems.add(sub);
        }
    }

    public void reset() {
        stop();
        for(NAR_Subsystem subsystem : subsystems) {
            subsystem.reset();
        }
    }

    public List<NAR_Subsystem> getSubsystems() {
        return subsystems;
    }

    public NAR_Subsystem getSubsystem(String name) {
        for(NAR_Subsystem subsystem : subsystems) {
            if(subsystem.getName().equals(name)) {
                return subsystem;
            }
        }
        return null;
    }

    public void setNeutralMode(Neutral mode) {
        getSubsystems().forEach(subsystem -> subsystem.setNeutralMode(mode));
    }

    public Command run(double power) {
        return runOnce(()-> {
            subsystems.forEach((subsystem)-> subsystem.run(power));
        });
    }

    public Command runVolts(double volts) {
        return runOnce(()-> {
            subsystems.forEach((subsystem)-> subsystem.runVolts(volts));
        });
    }

    public Command stop() {
        currentTransition.cancel();
        return runOnce(()-> {
            subsystems.forEach((subsystem)-> subsystem.stop());
        });
    }
}
