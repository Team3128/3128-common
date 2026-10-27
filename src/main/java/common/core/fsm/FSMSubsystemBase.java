package common.core.fsm;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;

import common.core.subsystems.MechanismBase;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class FSMSubsystemBase<S extends Enum<S>> extends SubsystemBase {
    
    protected Transition<S> currentTransition;
    protected S currentState;
    protected Transition<S> requestTransition;
    protected S previousState;

    private final TransitionMap<S> transitionMap;
    private final Class<S> enumType;

    protected List<MechanismBase> mechanisms = new LinkedList<MechanismBase>();

    public FSMSubsystemBase(Class<S> enumType, TransitionMap<S> transitionMap) {
        this.enumType = enumType;
        this.transitionMap = transitionMap;
    }

    public FSMSubsystemBase(Class<S> enumType, TransitionMap<S> transitionMap, S initalState) {
        this(enumType, transitionMap);
        this.currentState = initalState;
    }

    public void initShuffleboard() {
        NAR_Shuffleboard.addData(this.getName(), "Transition Count", ()-> transitionMap.getTransitionCount(), 0, 0);
        NAR_Shuffleboard.addData(this.getName(), "Previous State", ()-> {if(getPreviousState() != null) return getPreviousState().name(); else return "Null";}, 1, 0);
        NAR_Shuffleboard.addData(this.getName(), "Current State", ()-> {if(getState() != null) return getState().name(); else return "Null";}, 2, 0);
        NAR_Shuffleboard.addData(this.getName(), "Valid Transition", ()-> getRequestTransition() != null, 3, 0);
        for(S state : enumType.getEnumConstants()) {
            NAR_Shuffleboard.addData(this.getName(), state.name(), ()-> stateEquals(state), (state.ordinal() % 8), state.ordinal() / 8 + 1);
        }
    }

    public void overrideState(S nextState) {
        previousState = currentState;
        currentState = nextState;
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
        requestTransition = transition;

        // if invalid trnasition
        if(transition == null || transition.getCommand() == null) {
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
        CommandScheduler.getInstance().schedule(currentTransition.getCommand());
        previousState = currentState;
        currentState = nextState;
        return;
    }

    public Command setStateCommand(S nextState) {
        return Commands.runOnce(()-> setState(nextState));
    }

    public boolean stateEquals(S otherState) {
        return getState() == otherState;
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

    public abstract void registerTransitions();

    public void addMechanisms(MechanismBase... mechanisms) {
        this.mechanisms = List.of(mechanisms);
    }

    public void reset() {
        stop();
        for(MechanismBase subsystem : mechanisms) {
            subsystem.reset();
        }
    }

    public Command resetCommand() {
        return runOnce(()-> reset()).beforeStarting(()-> Log.debug(Log.Type.STATE_MACHINE_SECONDARY, getName(), "Commanded to reset"));
    }

    public List<MechanismBase> getMechanisms() {
        return mechanisms;
    }

    public MechanismBase getMechanism(String name) {
        for(MechanismBase subsystem : mechanisms) {
            if(subsystem.getName().equals(name)) {
                return subsystem;
            }
        }
        return null;
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
