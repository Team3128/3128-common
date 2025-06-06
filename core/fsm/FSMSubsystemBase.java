package common.core.fsm;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;

import common.core.subsystems.NAR_Subsystem;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class FSMSubsystemBase<S extends Enum<S>> extends SubsystemBase implements NAR_Subsystem {
    
    protected Transition<S> currentTransition;
    protected S currentState;
    protected Transition<S> requestTransition;
    protected S previousState;

    private final TransitionMap<S> transitionMap;
    private final Class<S> enumType;

    protected List<NAR_Subsystem> mechanisms = new LinkedList<NAR_Subsystem>();

    public FSMSubsystemBase(Class<S> enumType, TransitionMap<S> transitionMap) {
        this.enumType = enumType;
        this.transitionMap = transitionMap;
        // initShuffleboard();
        // try{
        //     registerTransitions();
        // } catch(Exception e) {
        //     Log.divider(10);
        //     Log.unusual(getName(), "Failed to load TransitionMap in constructor");
        //     Log.divider(10);
        // }
    }

    public FSMSubsystemBase(Class<S> enumType, TransitionMap<S> transitionMap, S initalState) {
        this(enumType, transitionMap);
        this.currentState = initalState;
    }

    @Override
    public void initShuffleboard() {
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
            Log.debug(Log.Type.STATE_MACHINE_PRIMARY, getName(), "Registering Transitions");
        }

        if(nextState == null) {
            Log.recoverable(getName(), "Null state requested");
            return;
        }

        Log.debug(Log.Type.STATE_MACHINE_PRIMARY, getName(), "Robot attempting to set state. FROM: " + currentState.name() + "  TO: " + nextState.name());
        Transition<S> transition = transitionMap.getTransition(getState(), nextState);
        
        // if not the same state
        if(!stateEquals(nextState)) requestTransition = transition;
        else {
            Log.debug(Log.Type.STATE_MACHINE_SECONDARY, getName(), "Invalid Transition: Requested state already reached");
            return;
        }

        // if invalid trnasition
        if(transition == null) {
            Log.unusual(getName(), "Invalid Transition: Requested transition null");
            return;
        }

        Log.debug(Log.Type.STATE_MACHINE_SECONDARY, getName(), "Valid Transition: " + transition);


        // if not transitioning
        if(isTransitioning()) {
            Log.debug(Log.Type.STATE_MACHINE_SECONDARY, getName(), "Canceling current transition...");
            currentTransition.cancel();
        }

        Log.debug(Log.Type.STATE_MACHINE_SECONDARY, getName(), "Scheduling transition...");
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
        if(currentState == null) return false;
        if(otherState == null) {
            Log.recoverable(getName(), "Null state passed");
            return false;
        }
        return currentState.name().equals(otherState.name()) && !isTransitioning();
    }

    @SuppressWarnings("unchecked")
    public boolean stateEquals(S... otherStates) {
        for(S otherState : otherStates) {
            if(stateEquals(otherState)) return true;
        }
        return false;
    }

    public boolean stateEquals(List<S> otherStates) {
        for(S otherState : otherStates) {
            if(stateEquals(otherState)) return true;
        }
        return false;
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

    public void addMechanisms(NAR_Subsystem... mechanisms) {
        this.mechanisms = List.of(mechanisms);
    }

    public void apply(Consumer<NAR_Subsystem> action) {
        mechanisms.forEach(action);
    }

    public Command applyCommand(Consumer<NAR_Subsystem> action) {
        return runOnce(()-> apply(action));
    }

    public void reset() {
        stop();
        for(NAR_Subsystem subsystem : mechanisms) {
            subsystem.reset();
        }
    }

    public Command resetCommand() {
        return runOnce(()-> reset()).beforeStarting(()-> Log.debug(Log.Type.STATE_MACHINE_SECONDARY, getName(), "Commanded to reset"));
    }

    public List<NAR_Subsystem> getMechanisms() {
        return mechanisms;
    }

    public NAR_Subsystem getMechanism(String name) {
        for(NAR_Subsystem subsystem : mechanisms) {
            if(subsystem.getName().equals(name)) {
                return subsystem;
            }
        }
        return null;
    }

    public void setNeutralMode(Neutral mode) {
        Log.debug(getName(), "Neutral Mode set to " + mode.name());
        getMechanisms().forEach(subsystem -> subsystem.setNeutralMode(mode));
    }

    public void run(double power) {
        mechanisms.forEach(subsystem-> subsystem.run(power));
    }

    public Command runCommand(double power) {
        return runOnce(()-> run(power)).beforeStarting(()-> Log.debug(Log.Type.STATE_MACHINE_SECONDARY, getName(), "Set to run at " + power + " power"));
    }

    public void runVolts(double volts) {
        mechanisms.forEach((subsystem)-> subsystem.runVolts(volts));
    }

    public Command runVoltsCommand(double volts) {
        return runOnce(()-> runVolts(volts)).beforeStarting(()-> Log.debug(Log.Type.STATE_MACHINE_SECONDARY, getName(), "Set to run at " + volts + " volts"));
    }

    public double getVolts() {
        return mechanisms.get(0).getVolts();
    }

    public void stop() {
        Log.info(getName(), "Disabling");
        if(currentTransition != null) currentTransition.cancel();
        mechanisms.forEach((subsystem)-> subsystem.stop());
    }

    public Command stopCommand() {
        return runOnce(()-> stop()).beforeStarting(()-> Log.debug(Log.Type.STATE_MACHINE_SECONDARY, getName(), "Commanded to Stop"));
    }
}
