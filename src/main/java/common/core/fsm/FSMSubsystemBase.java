package common.core.fsm;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;

import common.core.subsystems.MechanismBase;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Base class for a subsystem that is always in one state from a fixed list and changes state only
 * through the transitions registered in {@link #registerTransitions()}.
 *
 * <p>The current state is published to NetworkTables under {@code StateMachines/<name>} whenever
 * it changes. See {@code README.md} in this package for a walkthrough.
 *
 * @param <S> The enum listing this subsystem's states
 */
public abstract class FSMSubsystemBase<S extends Enum<S>> extends SubsystemBase {

    /** The transition that was last started; its command may still be running. */
    protected Transition<S> currentTransition;
    protected S currentState;
    /** The transition found for the last {@link #setState} request; null if that transition is not allowed. */
    protected Transition<S> requestTransition;
    protected S previousState;

    private final TransitionMap<S> transitionMap;
    private final Class<S> enumType;

    /** The mechanisms this subsystem controls, for {@link #stop()} and {@link #reset()}. */
    protected List<MechanismBase> mechanisms = new LinkedList<MechanismBase>();

    private StringPublisher statePublisher;

    /**
     * Creates a state machine with no initial state. The constructor that takes a starting state is
     * preferred, because {@link #setState} requires a current state from which to find a transition.
     *
     * @param enumType The states' enum class, for example, {@code IntakeStates.class}
     * @param transitionMap Holds the allowed transitions; filled in by {@link #registerTransitions()}
     */
    public FSMSubsystemBase(Class<S> enumType, TransitionMap<S> transitionMap) {
        this.enumType = enumType;
        this.transitionMap = transitionMap;
    }

    /**
     * Creates a state machine that starts in {@code initalState}.
     *
     * @param enumType The states' enum class, for example, {@code IntakeStates.class}
     * @param transitionMap Holds the allowed transitions; filled in by {@link #registerTransitions()}
     * @param initalState The state of the subsystem when the robot turns on
     */
    public FSMSubsystemBase(Class<S> enumType, TransitionMap<S> transitionMap, S initalState) {
        this(enumType, transitionMap);
        this.currentState = initalState;
        logState();
    }

    /**
     * Publishes the current state to NetworkTables under {@code StateMachines/<name>}, for
     * AdvantageScope and the robot's log file. Called on every state change.
     */
    private void logState() {
        // Created on first use, in case a subclass sets its name after this constructor runs
        if (statePublisher == null) {
            statePublisher = NetworkTableInstance.getDefault().getTable("StateMachines").getStringTopic(getName()).publish();
        }
        statePublisher.set(currentState != null ? currentState.name() : "None");
    }

    /**
     * Adds a Shuffleboard tab with the current and previous state, whether the last request was
     * allowed, and an indicator for each state, laid out eight to a row from the second row down.
     */
    public void initShuffleboard() {
        NAR_Shuffleboard.addData(this.getName(), "Transition Count", ()-> transitionMap.getTransitionCount(), 0, 0);
        NAR_Shuffleboard.addData(this.getName(), "Previous State", ()-> {if(getPreviousState() != null) return getPreviousState().name(); else return "Null";}, 1, 0);
        NAR_Shuffleboard.addData(this.getName(), "Current State", ()-> {if(getState() != null) return getState().name(); else return "Null";}, 2, 0);
        NAR_Shuffleboard.addData(this.getName(), "Valid Transition", ()-> getRequestTransition() != null, 3, 0);
        for(S state : enumType.getEnumConstants()) {
            NAR_Shuffleboard.addData(this.getName(), state.name(), ()-> stateEquals(state), (state.ordinal() % 8), state.ordinal() / 8 + 1);
        }
    }

    /**
     * Changes the state without running a transition or checking that the change is allowed. Use
     * this to correct the state after something outside the state machine has moved the mechanisms,
     * not to move them.
     */
    public void overrideState(S nextState) {
        previousState = currentState;
        currentState = nextState;
        logState();
    }

    /**
     * Moves to {@code nextState} by running the transition registered for it, if one exists.
     *
     * <ul>
     * <li>If the transition is not allowed, the state does not change and a warning is logged.</li>
     * <li>If a transition is still running, it is cancelled and the new transition starts.</li>
     * <li>The state changes as soon as the transition starts, not when its command finishes.
     *     Use {@link #isTransitioning()} to check whether the transition is still running.</li>
     * </ul>
     *
     * <p>The first call registers the transitions if {@link #registerTransitions()} has not
     * yet filled the map.
     */
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

        // Retained even when null, so the dashboard can show that the request was not allowed
        requestTransition = transition;

        if(transition == null || transition.getCommand() == null) {
            Log.unusual(getName(), "Invalid Transition: Requested transition null");
            return;
        }

        Log.debug(Log.Type.STATE_MACHINE_SECONDARY, getName(), "Valid Transition: " + transition);


        if(isTransitioning()) {
            Log.debug(Log.Type.STATE_MACHINE_SECONDARY, getName(), "Canceling current transition...");
            currentTransition.cancel();
        }

        Log.debug(Log.Type.STATE_MACHINE_SECONDARY, getName(), "Scheduling transition...");
        currentTransition = transition;
        CommandScheduler.getInstance().schedule(currentTransition.getCommand());
        previousState = currentState;
        currentState = nextState;
        logState();
        return;
    }

    /** Returns a command that calls {@link #setState(Enum)} once, for button bindings and autos. */
    public Command setStateCommand(S nextState) {
        return Commands.runOnce(()-> setState(nextState));
    }

    /** Returns true if the current state is {@code otherState}. */
    public boolean stateEquals(S otherState) {
        return getState() == otherState;
    }

    /** Returns true if the current state is any of the given states, for example, {@code stateEquals(SHOOT, RAM_SHOOT)}. */
    @SuppressWarnings("unchecked")
    public boolean stateEquals(S... otherStates) {
        for(S otherState : otherStates) {
            if(stateEquals(otherState)) return true;
        }
        return false;
    }

    public S getState() {
        return currentState;
    }

    /** Returns the state before the current one, or null until the state has changed once. */
    public S getPreviousState() {
        return previousState;
    }

    /** Returns the transition that was last started; its command may still be running. */
    public Transition<S> getCurrentTransition() {
        return currentTransition;
    }

    /** Returns the transition found for the last {@link #setState} request, or null if that transition was not allowed. */
    public Transition<S> getRequestTransition()  {
        return requestTransition;
    }

    /** Returns whether the last transition's command is still running. */
    public boolean isTransitioning() {
        if(currentTransition == null) return false;
        return currentTransition.isScheduled();
    }

    /**
     * Adds every allowed transition to the {@link TransitionMap}. Called automatically the first
     * time {@link #setState} is used, if the map is still empty.
     */
    public abstract void registerTransitions();

    /** Sets the mechanisms this subsystem controls, for {@link #stop()} and {@link #reset()}. */
    public void addMechanisms(MechanismBase... mechanisms) {
        this.mechanisms = List.of(mechanisms);
    }

    /** Stops all motion, then resets every mechanism's encoder to its starting position. */
    public void reset() {
        stop();
        for(MechanismBase subsystem : mechanisms) {
            subsystem.reset();
        }
    }

    /** Returns a command that calls {@link #reset()}, for example, for a button used after a mechanism slips. */
    public Command resetCommand() {
        return runOnce(()-> reset()).beforeStarting(()-> Log.debug(Log.Type.STATE_MACHINE_SECONDARY, getName(), "Commanded to reset"));
    }

    public List<MechanismBase> getMechanisms() {
        return mechanisms;
    }

    /** Returns the mechanism with the given name, or null if there is none. */
    public MechanismBase getMechanism(String name) {
        for(MechanismBase subsystem : mechanisms) {
            if(subsystem.getName().equals(name)) {
                return subsystem;
            }
        }
        return null;
    }

    /** Cancels the running transition and stops every mechanism. The state does not change. */
    public void stop() {
        Log.info(getName(), "Disabling");
        if(currentTransition != null) currentTransition.cancel();
        mechanisms.forEach((subsystem)-> subsystem.stop());
    }

    /** Returns a command that calls {@link #stop()}. */
    public Command stopCommand() {
        return runOnce(()-> stop()).beforeStarting(()-> Log.debug(Log.Type.STATE_MACHINE_SECONDARY, getName(), "Commanded to Stop"));
    }
}
