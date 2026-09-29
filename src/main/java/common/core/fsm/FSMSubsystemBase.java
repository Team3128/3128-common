package common.core.fsm;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BiFunction;

import common.core.subsystems.MechanismBase;
import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Base Finite State Machine (FSM) Subsystem supporting fully-connected transitions.
 *
 * @param <S> Enum type representing states
 */
public abstract class FSMSubsystemBase<S extends Enum<S>> extends SubsystemBase {

    private final Class<S> enumType;
    private final Map<S, Map<S, Command>> transitionTable;
    
    // Fallback factory used when no specific transition command is assigned
    private BiFunction<S, S, Command> defaultTransitionFactory = (from, to) -> Commands.none();

    protected S currentState;
    protected S previousState;
    protected Command currentCommand;

    protected List<MechanismBase> mechanisms = new ArrayList<>();

    public FSMSubsystemBase(Class<S> enumType) {
        this(enumType, null);
    }

    public FSMSubsystemBase(Class<S> enumType, S initialState) {
        this.enumType = Objects.requireNonNull(enumType, "Enum type cannot be null");
        this.currentState = initialState;
        this.transitionTable = new EnumMap<>(enumType);

        // Initialize table for every state pair
        for (S state : enumType.getEnumConstants()) {
            transitionTable.put(state, new EnumMap<>(enumType));
        }
    }

    // =========================================================================
    // Transition Registration & Universal Configuration
    // =========================================================================

    /** Must register transitions using helper methods within this implementation. */
    public abstract void registerTransitions();

    /**
     * Sets a global fallback action to execute whenever transitioning between any 
     * two states that don't have an explicitly defined command.
     */
    public void setDefaultTransitionAction(BiFunction<S, S, Command> transitionFactory) {
        if (transitionFactory != null) {
            this.defaultTransitionFactory = transitionFactory;
        }
    }

    /**
     * Explicitly registers a universal command for all state transitions across the FSM.
     */
    public void allowAllTransitions(Command universalCommand) {
        S[] states = enumType.getEnumConstants();
        for (S from : states) {
            for (S to : states) {
                if (from != to) {
                    addTransition(from, to, universalCommand);
                }
            }
        }
    }

    public void addTransition(S from, S to, Command command) {
        if (from == to) return;
        transitionTable.get(from).put(to, command != null ? command : Commands.none());
    }

    public void addTransition(S from, S to, Runnable action) {
        addTransition(from, to, Commands.runOnce(action));
    }

    // =========================================================================
    // FSM Core Execution Logic
    // =========================================================================

    public void setState(S nextState) {
        if (nextState == null) {
            Log.recoverable(getName(), "Null state requested");
            return;
        }

        if (currentState == nextState) {
            return;
        }

        Log.debug(Log.Type.STATE_MACHINE_PRIMARY, getName(), 
                "Attempting state transition. FROM: " + (currentState != null ? currentState.name() : "NONE") + " TO: " + nextState.name());

        Command transitionCmd = getTransitionCommand(currentState, nextState);

        // Cancel running transition if ongoing
        if (isTransitioning()) {
            Log.debug(Log.Type.STATE_MACHINE_SECONDARY, getName(), "Canceling active transition execution...");
            currentCommand.cancel();
        }

        currentCommand = transitionCmd;
        CommandScheduler.getInstance().schedule(currentCommand);

        previousState = currentState;
        currentState = nextState;
    }

    /**
     * Retrieves the mapped command for (from -> to). If no explicit command is set,
     * it falls back to the default transition factory so every state can transition to every other state.
     */
    public Command getTransitionCommand(S from, S to) {
        if (from == null || to == null) return Commands.none();
        
        Command registeredCommand = transitionTable.get(from).get(to);
        if (registeredCommand != null) {
            return registeredCommand;
        }

        // Fallback allows transition between any remaining state pair
        return defaultTransitionFactory.apply(from, to);
    }

    public Command setStateCommand(S nextState) {
        return Commands.runOnce(() -> setState(nextState));
    }

    public void overrideState(S nextState) {
        previousState = currentState;
        currentState = nextState;
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

    public boolean isTransitioning() {
        return currentCommand != null && currentCommand.isScheduled();
    }

    // =========================================================================
    // Subsystem & Mechanism Management
    // =========================================================================

    public void addMechanisms(MechanismBase... mechanisms) {
        Collections.addAll(this.mechanisms, mechanisms);
    }

    public List<MechanismBase> getMechanisms() {
        return Collections.unmodifiableList(mechanisms);
    }

    public MechanismBase getMechanism(String name) {
        for (MechanismBase mech : mechanisms) {
            if (mech.getName().equals(name)) return mech;
        }
        return null;
    }

    public void stop() {
        Log.info(getName(), "Disabling Subsystem");
        if (currentCommand != null && currentCommand.isScheduled()) {
            currentCommand.cancel();
        }
        mechanisms.forEach(MechanismBase::stop);
    }

    public Command stopCommand() {
        return runOnce(this::stop).beforeStarting(() -> 
                Log.debug(Log.Type.STATE_MACHINE_SECONDARY, getName(), "Commanded to Stop"));
    }

    public void reset() {
        stop();
        mechanisms.forEach(MechanismBase::reset);
    }

    public Command resetCommand() {
        return runOnce(this::reset).beforeStarting(() -> 
                Log.debug(Log.Type.STATE_MACHINE_SECONDARY, getName(), "Commanded to Reset"));
    }

    // =========================================================================
    // Dashboard & Logging
    // =========================================================================

    public void initShuffleboard() {
        NAR_Shuffleboard.addData(getName(), "Previous State", 
                () -> getPreviousState() != null ? getPreviousState().name() : "Null", 1, 0);
        NAR_Shuffleboard.addData(getName(), "Current State", 
                () -> getState() != null ? getState().name() : "Null", 2, 0);
        NAR_Shuffleboard.addData(getName(), "Is Transitioning", 
                this::isTransitioning, 3, 0);

        for (S state : enumType.getEnumConstants()) {
            NAR_Shuffleboard.addData(getName(), state.name(), 
                    () -> stateEquals(state), (state.ordinal() % 8), (state.ordinal() / 8) + 1);
        }
    }
}