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
 * A subsystem that is always in one of a fixed list of states, like NEUTRAL, INTAKE or SHOOT,
 * and only moves between them along the transitions you allow.
 *
 * <p>To make one:
 * <ol>
 * <li>List the states in an enum.</li>
 * <li>Extend this class and build its mechanisms in the constructor.</li>
 * <li>In {@link #registerTransitions()}, add a {@link Transition} for every move between states
 *     that's allowed, with the command that does it.</li>
 * </ol>
 *
 * <pre>{@code
 * public class Intake extends FSMSubsystemBase<IntakeStates> {
 *     private static final TransitionMap<IntakeStates> transitionMap = new TransitionMap<>(IntakeStates.class);
 *
 *     public Intake() {
 *         super(IntakeStates.class, transitionMap, IntakeStates.NEUTRAL);
 *         roller = new RollerMechanism(...);
 *         addMechanisms(roller);
 *     }
 *
 *     public void registerTransitions() {
 *         // Any state can go to any other, running the roller at that state's power
 *         transitionMap.addCommutativeTransition(List.of(IntakeStates.values()),
 *             state -> roller.runCommand(state.getPower()));
 *     }
 * }
 * }</pre>
 *
 * <p>Then {@code intake.setState(IntakeStates.INTAKE)} runs the transition's command. Asking
 * for a move that isn't in the map does nothing and logs a warning, which keeps a subsystem from
 * doing something unplanned, like extending an arm through the robot.
 *
 * <p>The current state is logged to NetworkTables under {@code StateMachines/<name>} whenever
 * it changes, so you can follow it in AdvantageScope or a log file.
 *
 * @param <S> The enum listing this subsystem's states
 */
public abstract class FSMSubsystemBase<S extends Enum<S>> extends SubsystemBase {

    /** The transition that was last started; its command may still be running. */
    protected Transition<S> currentTransition;
    protected S currentState;
    /** The transition found for the last {@link #setState} request; null if that move isn't allowed. */
    protected Transition<S> requestTransition;
    protected S previousState;

    private final TransitionMap<S> transitionMap;
    private final Class<S> enumType;

    /** The mechanisms this subsystem controls, for {@link #stop()} and {@link #reset()}. */
    protected List<MechanismBase> mechanisms = new LinkedList<MechanismBase>();

    private StringPublisher statePublisher;

    /**
     * Creates a state machine with no state yet. Prefer the constructor that takes a starting
     * state, since {@link #setState} needs a current state to find a transition from.
     *
     * @param enumType The states' enum class, e.g. {@code IntakeStates.class}
     * @param transitionMap Holds the allowed transitions; filled in by {@link #registerTransitions()}
     */
    public FSMSubsystemBase(Class<S> enumType, TransitionMap<S> transitionMap) {
        this.enumType = enumType;
        this.transitionMap = transitionMap;
    }

    /**
     * Creates a state machine that starts in {@code initalState}.
     *
     * @param enumType The states' enum class, e.g. {@code IntakeStates.class}
     * @param transitionMap Holds the allowed transitions; filled in by {@link #registerTransitions()}
     * @param initalState The state it's in when the robot turns on
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
     * allowed, and a light for every state that's on when the subsystem is in it.
     */
    public void initShuffleboard() {
        NAR_Shuffleboard.addData(this.getName(), "Transition Count", ()-> transitionMap.getTransitionCount(), 0, 0);
        NAR_Shuffleboard.addData(this.getName(), "Previous State", ()-> {if(getPreviousState() != null) return getPreviousState().name(); else return "Null";}, 1, 0);
        NAR_Shuffleboard.addData(this.getName(), "Current State", ()-> {if(getState() != null) return getState().name(); else return "Null";}, 2, 0);
        NAR_Shuffleboard.addData(this.getName(), "Valid Transition", ()-> getRequestTransition() != null, 3, 0);
        // One light per state, 8 to a row, starting on the second row
        for(S state : enumType.getEnumConstants()) {
            NAR_Shuffleboard.addData(this.getName(), state.name(), ()-> stateEquals(state), (state.ordinal() % 8), state.ordinal() / 8 + 1);
        }
    }

    /**
     * Changes the state without running any transition or checking it's allowed. Use it to
     * correct the state after something outside the state machine moved the mechanisms, not to
     * move them yourself.
     */
    public void overrideState(S nextState) {
        previousState = currentState;
        currentState = nextState;
        logState();
    }

    /**
     * Moves to {@code nextState} by running the transition registered for it, if there is one.
     *
     * <ul>
     * <li>If the move isn't allowed, nothing changes and a warning is logged.</li>
     * <li>If a transition is still running, it's cancelled and the new one starts.</li>
     * <li>The state changes as soon as the transition starts, not when its command finishes.
     *     Use {@link #isTransitioning()} to check whether it's still moving.</li>
     * </ul>
     *
     * <p>The first call registers the transitions, if {@link #registerTransitions()} hasn't
     * filled the map yet.
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

        // Remembered even when null, so the dashboard can show the request wasn't allowed
        requestTransition = transition;

        // Not an allowed move, so stay put
        if(transition == null || transition.getCommand() == null) {
            Log.unusual(getName(), "Invalid Transition: Requested transition null");
            return;
        }

        Log.debug(Log.Type.STATE_MACHINE_SECONDARY, getName(), "Valid Transition: " + transition);


        // A new request takes over from one that hasn't finished
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

    /** A command that calls {@link #setState(Enum)} once, for button bindings and autos. */
    public Command setStateCommand(S nextState) {
        return Commands.runOnce(()-> setState(nextState));
    }

    /** True if the current state is {@code otherState}. */
    public boolean stateEquals(S otherState) {
        return getState() == otherState;
    }

    /** True if the current state is any of the given states, e.g. {@code stateEquals(SHOOT, RAM_SHOOT)}. */
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

    /** The state before the current one; null until the state has changed once. */
    public S getPreviousState() {
        return previousState;
    }

    /** The transition that was last started; its command may still be running. */
    public Transition<S> getCurrentTransition() {
        return currentTransition;
    }

    /** The transition found for the last {@link #setState} request; null if that move wasn't allowed. */
    public Transition<S> getRequestTransition()  {
        return requestTransition;
    }

    /** Whether the last transition's command is still running. */
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

    /** Stops everything, then resets every mechanism's encoder to its starting position. */
    public void reset() {
        stop();
        for(MechanismBase subsystem : mechanisms) {
            subsystem.reset();
        }
    }

    /** A command that calls {@link #reset()}, e.g. for a button after something slips. */
    public Command resetCommand() {
        return runOnce(()-> reset()).beforeStarting(()-> Log.debug(Log.Type.STATE_MACHINE_SECONDARY, getName(), "Commanded to reset"));
    }

    public List<MechanismBase> getMechanisms() {
        return mechanisms;
    }

    /** Finds one of its mechanisms by name; null if there isn't one. */
    public MechanismBase getMechanism(String name) {
        for(MechanismBase subsystem : mechanisms) {
            if(subsystem.getName().equals(name)) {
                return subsystem;
            }
        }
        return null;
    }

    /** Cancels the running transition and stops every mechanism. The state doesn't change. */
    public void stop() {
        Log.info(getName(), "Disabling");
        if(currentTransition != null) currentTransition.cancel();
        mechanisms.forEach((subsystem)-> subsystem.stop());
    }

    /** A command that calls {@link #stop()}. */
    public Command stopCommand() {
        return runOnce(()-> stop()).beforeStarting(()-> Log.debug(Log.Type.STATE_MACHINE_SECONDARY, getName(), "Commanded to Stop"));
    }
}
