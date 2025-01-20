package common.core.fsm;

import java.util.LinkedList;
import java.util.List;

import common.core.subsystems.NAR_Subsystem;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class FSMSubsystemBase<S extends Enum<S>> extends SubsystemBase {
    
    protected Transition<S> currentTransition;
    protected S currentState;
    protected Transition<S> requestTransition;
    protected S previousState;

    private final TransitionMap<S> transitionMap;
    private final Class<S> enumType;

    protected static List<NAR_Subsystem> subsystems = new LinkedList<NAR_Subsystem>();

    public FSMSubsystemBase(Class<S> enumType, TransitionMap<S> transitionMap) {
        this.enumType = enumType;
        this.transitionMap = transitionMap;
        initStateTracker();
        try{
            registerTransitions();
        } catch(Exception e) {
            Log.divider(10);
            Log.recoverable(getName(), "Failed to load TransitionMap in constructor");
            Log.divider(10);
        }
    }

    public FSMSubsystemBase(Class<S> enumType, TransitionMap<S> transitionMap, S initalState) {
        this(enumType, transitionMap);
        this.currentState = initalState;
    }

    public void initStateTracker() {
        NAR_Shuffleboard.addData(this.getName(), "Transition Count", ()-> transitionMap.getTransitionCount(), 0, 0);
        NAR_Shuffleboard.addData(this.getName(), "Previous State", ()-> {if(getPreviousState() != null) return getPreviousState().name(); else return "Null";}, 1, 0);
        NAR_Shuffleboard.addData(this.getName(), "Current State", ()-> {if(getState() != null) return getState().name(); else return "Null";}, 2, 0);
        NAR_Shuffleboard.addData(this.getName(), "Valid Transition", ()-> getRequestTransition() != null, 3, 0);
        for(S state : enumType.getEnumConstants()) {
            NAR_Shuffleboard.addData(this.getName(), state.name(), ()-> stateEquals(state), (state.ordinal() % 8), state.ordinal() / 8 + 1);
        }
    }
    
    public void setState(S nextState) {
        if(transitionMap.isEmpty()) {
            registerTransitions();
            Log.debug(Log.Type.STATE_MACHINE, getName(), "Registering Transitions");
        }

        if(nextState == null) {
            Log.recoverable(getName(), "Null state requested");
            return;
        }

        Log.debug(Log.Type.STATE_MACHINE, getName(), "Robot attempting to set state. \n\tFROM: " + currentState.name() + "\n\t  TO: " + nextState.name());
        Transition<S> transition = transitionMap.getTransition(getState(), nextState);
        
        // if not the same state
        if(!stateEquals(nextState)) requestTransition = transition;
        else {
            Log.recoverable(getName(), "Invalid Transition: Requested state already reached");
            return;
        }

        // if invalid trnasition
        if(transition == null) {
            Log.recoverable(getName(), "Invalid Transition: Requested transition null");
            return;
        }

        Log.debug(Log.Type.STATE_MACHINE, getName(), "Valid Transition: " + transition);


        // if not transitioning
        if(isTransitioning()) {
            Log.debug(Log.Type.STATE_MACHINE, getName(), "Canceling current transition...");
            currentTransition.cancel();
        }

        Log.debug(Log.Type.STATE_MACHINE, getName(), "Scheduling transition...");
        currentTransition = transition;
        currentTransition.getCommand().schedule();
        previousState = currentState;
        currentState = nextState;
        return;
    }

    public Command setStateCommand(S nextState) {
        return Commands.runOnce(()-> setState(nextState));
    }

    public boolean stateEquals(S otherState) {
        if(otherState == null || currentState == null) {
            Log.recoverable(getName(), "Null state passed");
            return false;
        }
        return currentState.name().equals(otherState.name()) && !isTransitioning();
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
        if(currentTransition == null) return false;
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
        Log.debug(getName(), "Neutral Mode set to " + mode.name());
        getSubsystems().forEach(subsystem -> subsystem.setNeutralMode(mode));
    }

    public Command run(double power) {
        return runOnce(()-> {
            subsystems.forEach((subsystem)-> subsystem.run(power));
        }).beforeStarting(runOnce(()-> Log.debug(Log.Type.STATE_MACHINE, getName(), "Set to run at " + power + " power")));
    }

    public Command runVolts(double volts) {
        return runOnce(()-> {
            subsystems.forEach((subsystem)-> subsystem.runVolts(volts));
        }).beforeStarting(runOnce(()-> Log.debug(Log.Type.STATE_MACHINE, getName(), "Set to run at " + volts + " volts")));
    }

    public Command stop() {
        if(currentTransition != null) currentTransition.cancel();
        return runOnce(()-> {
            subsystems.forEach((subsystem)-> subsystem.stop());
        });
    }
}
