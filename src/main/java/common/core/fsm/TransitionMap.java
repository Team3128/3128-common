package common.core.fsm;
import java.util.*;
import java.util.function.Function;

import common.utility.Log;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

/**
 * Every allowed move between the states of an {@link FSMSubsystemBase}, and the command each one
 * runs. Moves that aren't in the map aren't allowed.
 *
 * <p>Adding transitions one at a time gets long, so there are shortcuts for common patterns:
 * <ul>
 * <li><b>addTransition</b>: one state to another, e.g. NEUTRAL to INTAKE.</li>
 * <li><b>addCommutativeTransition</b>: both ways between states, e.g. NEUTRAL to INTAKE and back.
 *     Given one list, every state in it to every other.</li>
 * <li><b>addConvergingTransition</b>: from many states (all of them, if you don't list any) into
 *     one, e.g. anything to NEUTRAL.</li>
 * <li><b>addDivergingTransition</b>: from one state out to many (all of them, if you don't list
 *     any), e.g. NEUTRAL to anything.</li>
 * <li><b>addCorrespondenceTransitions</b>: pairs states from two lists in order: the first of
 *     each, then the second of each, and so on.</li>
 * <li><b>addMappedTransition</b>: a list of (from, to) pairs.</li>
 * <li><b>addUndefinedState</b>: a state, e.g. a manual override, that anything can go into but
 *     only leaves to one exit state.</li>
 * </ul>
 *
 * <p>Each comes in versions that differ only in what the transitions do:
 * <ul>
 * <li>nothing: only the state changes</li>
 * <li>a {@code Runnable}: run once, e.g. {@code () -> roller.run(0.5)}</li>
 * <li>a {@code Command}: the same command for every transition</li>
 * <li>a {@code Function<S, Command>}: builds the command for the state being entered, e.g.
 *     {@code state -> arm.setSetpointCommand(state.getAngle())}, so one line covers states that
 *     each need a different setpoint</li>
 * </ul>
 *
 * <p>Adding a transition between two states that already have one replaces it, and transitions
 * from a state to itself are ignored.
 *
 * <p>Based on ShamLib by team 5907.
 *
 * @author Teja Yaramada
 *
 * @param <S> The enum listing the states
 */
public class TransitionMap<S extends Enum<S>> {

    // adjacencyMap[from][to] is the transition between the states with those ordinals, or null
    private final Object[][] adjacencyMap;
    @SuppressWarnings("unused")
    private final Class<S> enumType;
    private final List<S> states;

    /** @param enumType The states' enum class, e.g. {@code IntakeStates.class} */
    public TransitionMap(Class<S> enumType) {
        this.enumType = enumType;
        this.states = List.of(enumType.getEnumConstants());


        adjacencyMap = new Object[states.size()][states.size()];
    }

    /**
     * @return Whether no transitions have been added yet
     */
    public boolean isEmpty() {
        return getTransitionCount() == 0;
    }

    /**
     * Counts the transitions in the map. Counted fresh each time, so adding a transition twice or
     * removing one that isn't there can't throw the count off.
     */
    public int getTransitionCount() {
        int transitionCount = 0;
        for(Object[] row : adjacencyMap){
            for(Object transition : row) {
                if(transition != null) transitionCount++;
            }
        }
        return transitionCount;
    }

    // ---- One state to another ----

    /**
     * Adds a transition, replacing any existing one between the same two states. Transitions
     * from a state to itself are ignored.
     *
     * @param transition The transition to add
     */
    public void addTransition(Transition<S> transition) {
        setTransition(transition);
    }

    /** Adds several transitions. */
    @SuppressWarnings("unchecked")
    public void addTransitions(Transition<S>... transitions) {
        for (Transition<S> transition : transitions) {
            addTransition(transition);
        }
    }

    /** From {@code start} to {@code end}, running {@code runnable} once. */
    public void addTransition(S start, S end, Runnable runnable) {
        addTransition(start, end, runOnce(runnable));
    }

    /** From {@code start} to {@code end}, running {@code command}. */
    public void addTransition(S start, S end, Command command) {
        addTransition(new Transition<S>(start, end, command));
    }

    /** From {@code start} to {@code end}, running the command {@code function} builds for {@code end}. */
    public void addTransition(S start, S end, Function<S, Command> function){
        addTransition(start, end, function.apply(end));
    }



    // ---- Pairs from two lists, in order ----

    /** From each state in {@code start} to the state at the same place in {@code end}. Does nothing. */
    public void addCorrespondenceTransitions(List<S> start, List<S> end) {
        addCorrespondenceTransitions(start, end, none());
    }

    /** From each state in {@code start} to the state at the same place in {@code end}, running {@code runnable}. */
    public void addCorrespondenceTransitions(List<S> start, List<S> end, Runnable runnable) {
        addCorrespondenceTransitions(start, end, runOnce(runnable));
    }

    /** From each state in {@code start} to the state at the same place in {@code end}, running {@code command}. */
    public void addCorrespondenceTransitions(List<S> start, List<S> end, Command command) {
        addCorrespondenceTransitions(start, end, state -> {return command;});
    }

    /**
     * From each state in {@code start} to the state at the same place in {@code end}, running the
     * command {@code function} builds for the state being entered. The lists must be the same length.
     */
    public void addCorrespondenceTransitions(List<S> start, List<S> end, Function<S, Command> function) {
        if(start.size() != end.size()) Log.fatal("Transition Map", "Failed to load one to one transition");
        for(int i = 0; i < end.size(); i++) {
            addTransition(start.get(i), end.get(i), function);
        }
    }



    // ---- Many states into one ----

    /** From every state into {@code end}. Does nothing. */
    public void addConvergingTransition(S end) {
        addConvergingTransition(end, none());
    }

    /** From every state into {@code end}, running {@code runnable}. */
    public void addConvergingTransition(S end, Runnable runnable) {
        addConvergingTransition(end, runOnce(runnable));
    }

    /** From every state into {@code end}, running {@code command}. */
    public void addConvergingTransition(S end, Command command) {
        for (S start : states) {
            addTransition(start, end, command);
        }
    }

    /** From every state into {@code end}, running the command {@code function} builds for {@code end}. */
    public void addConvergingTransition(S end, Function<S, Command> function) {
        addConvergingTransition(end, function.apply(end));
    }

    /** From each state in {@code start} into {@code end}. Does nothing. */
    public void addConvergingTransition(List<S> start, S end) {
        addConvergingTransition(start, end, none());
    }

    /** From each state in {@code start} into {@code end}, running {@code runnable}. */
    public void addConvergingTransition(List<S> start, S end, Runnable runnable) {
        addConvergingTransition(start, end, runOnce(runnable));
    }

    /** From each state in {@code start} into {@code end}, running {@code command}. */
    public void addConvergingTransition(List<S> start, S end, Command command) {
        for (S outGoingState : start) {
            addTransition(outGoingState, end, command);
        }
    }

    /** From each state in {@code start} into {@code end}, running the command {@code function} builds for {@code end}. */
    public void addConvergingTransition(List<S> start, S end, Function<S, Command> function) {
        addConvergingTransition(start, end, function.apply(end));
    }

    /** From every state into each state in {@code end}. Does nothing. */
    public void addConvergingTransition(List<S> end) {
        addConvergingTransition(end, none());
    }

    /** From every state into each state in {@code end}, running {@code runnable}. */
    public void addConvergingTransition(List<S> end, Runnable runnable) {
        addConvergingTransition(end, runOnce(runnable));
    }

    /** From every state into each state in {@code end}, running {@code command}. */
    public void addConvergingTransition(List<S> end, Command command) {
        for(S incomingState : end) {
            addConvergingTransition(incomingState, command);
        }
    }

    /** From every state into each state in {@code end}, running the command {@code function} builds for the state being entered. */
    public void addConvergingTransition(List<S> end, Function<S, Command> function) {
        for (S incomingState : end) {
           addConvergingTransition(incomingState, function.apply(incomingState));
        }
    }

    /** From each state in {@code start} into each state in {@code end}. Does nothing. */
    public void addConvergingTransition(List<S> start, List<S> end) {
        addConvergingTransition(start, end, none());
    }

    /** From each state in {@code start} into each state in {@code end}, running {@code runnable}. */
    public void addConvergingTransition(List<S> start, List<S> end, Runnable runnable) {
        addConvergingTransition(start, end, runOnce(runnable));
    }

    /** From each state in {@code start} into each state in {@code end}, running {@code command}. */
    public void addConvergingTransition(List<S> start, List<S> end, Command command) {
        for(S incomingState : end) {
            addConvergingTransition(start, incomingState, command);
        }
    }

    /** From each state in {@code start} into each state in {@code end}, running the command {@code function} builds for the state being entered. */
    public void addConvergingTransition(List<S> start, List<S> end, Function<S, Command> function) {
        for(S incomingState : end) {
            addConvergingTransition(start, incomingState, function.apply(incomingState));
        }
    }



    // ---- One state out to many ----

    /** From {@code start} to every other state. Does nothing. */
    public void addDivergingTransition(S start) {
        addDivergingTransition(start, none());
    }

    /** From {@code start} to every other state, running {@code runnable}. */
    public void addDivergingTransition(S start, Runnable runnable) {
        addDivergingTransition(start, runOnce(runnable));
    }

    /** From {@code start} to every other state, running {@code command}. */
    public void addDivergingTransition(S start, Command command) {
        addDivergingTransition(start, states, command);
    }

    /** From {@code start} to every other state, running the command {@code function} builds for the state being entered. */
    public void addDivergingTransition(S start, Function<S, Command> function) {
        addDivergingTransition(start, states, function);
    }

    /** From {@code start} to each state in {@code end}. Does nothing. */
    public void addDivergingTransition(S start, List<S> end) {
        addDivergingTransition(start, end, none());
    }

    /** From {@code start} to each state in {@code end}, running {@code runnable}. */
    public void addDivergingTransition(S start, List<S> end, Runnable runnable) {
        addDivergingTransition(start, end, runOnce(runnable));
    }

    /** From {@code start} to each state in {@code end}, running {@code command}. */
    public void addDivergingTransition(S start, List<S> end, Command command) {
        for (S incomingState : end) {
            addTransition(start, incomingState, command);
        }
    }

    /** From {@code start} to each state in {@code end}, running the command {@code function} builds for the state being entered. */
    public void addDivergingTransition(S start, List<S> end, Function<S, Command> function) {
        for (S incomingState : end) {
            addTransition(start, incomingState, function.apply(incomingState));
        }
    }

    /** From each state in {@code start} to every other state. Does nothing. */
    public void addDivergingTransition(List<S> start) {
        addDivergingTransition(start, none());
    }

    /** From each state in {@code start} to every other state, running {@code runnable}. */
    public void addDivergingTransition(List<S> start, Runnable runnable) {
        addDivergingTransition(start, runOnce(runnable));
    }

    /** From each state in {@code start} to every other state, running {@code command}. */
    public void addDivergingTransition(List<S> start, Command command) {
        for (S outgoingState : start) {
            addDivergingTransition(outgoingState, command);
        }
    }

    /** From each state in {@code start} to every other state, running the command {@code function} builds for the state being entered. */
    public void addDivergingTransition(List<S> start, Function<S, Command> function) {
        for (S outgoingState : start) {
            addDivergingTransition(outgoingState, function);
        }
    }

    /** From each state in {@code start} to each state in {@code end}. Does nothing. */
    public void addDivergingTransition(List<S> start, List<S> end) {
        addDivergingTransition(start, end, none());
    }

    /** From each state in {@code start} to each state in {@code end}, running {@code runnable}. */
    public void addDivergingTransition(List<S> start, List<S> end, Runnable runnable) {
        addDivergingTransition(start, end, runOnce(runnable));
    }

    /** From each state in {@code start} to each state in {@code end}, running {@code command}. */
    public void addDivergingTransition(List<S> start, List<S> end, Command command) {
        for (S outgoingState : start) {
            addDivergingTransition(outgoingState, end, command);
        }
    }

    /** From each state in {@code start} to each state in {@code end}, running the command {@code function} builds for the state being entered. */
    public void addDivergingTransition(List<S> start, List<S> end, Function<S, Command> function) {
        for (S outgoingState : start) {
            addDivergingTransition(outgoingState, end, function);
        }
    }



    // ---- Both ways ----

    /** Between {@code start} and {@code end}, both ways. Does nothing. */
    public void addCommutativeTransition(S start, S end) {
        addCommutativeTransition(start, end, none());
    }

    /** Between {@code start} and {@code end}, both ways, running {@code runnable}. */
    public void addCommutativeTransition(S start, S end, Runnable runnable) {
        addCommutativeTransition(start, end, runOnce(runnable));
    }

    /** Between {@code start} and {@code end}: {@code runnableForwards} going to {@code end}, {@code runnableBackwards} coming back. */
    public void addCommutativeTransition(S start, S end, Runnable runnableForwards, Runnable runnableBackwards) {
        addCommutativeTransition(start, end, runOnce(runnableForwards), runOnce(runnableBackwards));
    }

    /** Between {@code start} and {@code end}, both ways, running {@code command}. */
    public void addCommutativeTransition(S start, S end, Command command) {
        addCommutativeTransition(start, end, command, command);
    }

    /** Between {@code start} and {@code end}: {@code commandForwards} going to {@code end}, {@code commandBackwards} coming back. */
    public void addCommutativeTransition(S start, S end, Command commandForwards, Command commandBackwards) {
        addTransition(start, end, commandForwards);
        addTransition(end, start, commandBackwards);
    }

    /** Between {@code start} and {@code end}, both ways, each running the command {@code function} builds for the state being entered. */
    public void addCommutativeTransition(S start, S end, Function<S, Command> function) {
        addTransition(start, end, function);
        addTransition(end, start, function);
    }

    /** Between each state in {@code start} and {@code end}, both ways. Does nothing. */
    public void addCommutativeTransition(List<S> start, S end) {
        addCommutativeTransition(start, end, none());
    }

    /** Between each state in {@code start} and {@code end}, both ways, running {@code runnable}. */
    public void addCommutativeTransition(List<S> start, S end, Runnable runnable) {
        addCommutativeTransition(start, end, runOnce(runnable));
    }

    /** Between each state in {@code start} and {@code end}: {@code runnableForwards} going to {@code end}, {@code runnableBackwards} coming back. */
    public void addCommutativeTransition(List<S> start, S end, Runnable runnableForwards, Runnable runnableBackwards) {
        addCommutativeTransition(start, end, runOnce(runnableForwards), runOnce(runnableBackwards));
    }

    /** Between each state in {@code start} and {@code end}, both ways, running {@code command}. */
    public void addCommutativeTransition(List<S> start, S end, Command command) {
        addCommutativeTransition(start, end, command, command);
    }

    /** Between each state in {@code start} and {@code end}: {@code commandForwards} going to {@code end}, {@code commandBackwards} coming back. */
    public void addCommutativeTransition(List<S> start, S end, Command commandForwards, Command commandBackwards) {
        for (S outgoingState : start) {
            addCommutativeTransition(outgoingState, end, commandForwards, commandBackwards);
        }
    }

    /**
     * Between each state in {@code start} and {@code end}, both ways, running the command
     * {@code function} builds for {@code end}. Note: unlike the other function versions, coming
     * back from {@code end} also runs {@code end}'s command, not the command for the state entered.
     */
    public void addCommutativeTransition(List<S> start, S end, Function<S, Command> function) {
        addCommutativeTransition(start, end, function.apply(end));
    }

    /** Between {@code start} and each state in {@code end}, both ways. Does nothing. */
    public void addCommutativeTransition(S start, List<S> end) {
        addCommutativeTransition(start, end, none());
    }

    /** Between {@code start} and each state in {@code end}, both ways, running {@code runnable}. */
    public void addCommutativeTransition(S start, List<S> end, Runnable runnable) {
        addCommutativeTransition(start, end, runOnce(runnable));
    }

    /** Between {@code start} and each state in {@code end}: {@code runnableForwards} going out, {@code runnableBackwards} coming back. */
    public void addCommutativeTransition(S start, List<S> end, Runnable runnableForwards, Runnable runnableBackwards) {
        addCommutativeTransition(start, end, runOnce(runnableForwards), runOnce(runnableBackwards));
    }

    /** Between {@code start} and each state in {@code end}, both ways, running {@code command}. */
    public void addCommutativeTransition(S start, List<S> end, Command command) {
        addCommutativeTransition(start, end, command, command);
    }

    /** Between {@code start} and each state in {@code end}: {@code commandForwards} going out, {@code commandBackwards} coming back. */
    public void addCommutativeTransition(S start, List<S> end, Command commandForwards, Command commandBackwards) {
        for (S incomingState : end) {
            addCommutativeTransition(start, incomingState, commandForwards, commandBackwards);
        }
    }

    /**
     * Between {@code start} and each state in {@code end}, both ways. Each pair runs the command
     * {@code function} builds for the state in {@code end}, in both directions.
     */
    public void addCommutativeTransition(S start, List<S> end, Function<S, Command> function) {
        for (S incomingState : end) {
            addCommutativeTransition(start, incomingState, function.apply(incomingState));
        }
    }

    /** Between every pair of states in {@code states}, both ways. Does nothing. */
    public void addCommutativeTransition(List<S> states) {
        addCommutativeTransition(states, none());
    }

    /** Between every pair of states in {@code states}, both ways, running {@code runnable}. */
    public void addCommutativeTransition(List<S> states, Runnable runnable) {
        addCommutativeTransition(states, runOnce(runnable));
    }

    /** Between every pair of states in {@code states}, both ways, running {@code command}. */
    public void addCommutativeTransition(List<S> states, Command command) {
        addCommutativeTransition(states, state -> {return command;});
    }

    /**
     * Between every pair of states in {@code states}, both ways, each running the command
     * {@code function} builds for the state being entered. Handy when every state has its own
     * setpoints, e.g. {@code addCommutativeTransition(List.of(States.values()), state -> ...)}.
     */
    public void addCommutativeTransition(List<S> states, Function<S, Command> function) {
        for(S incomingState : states) {
            for(S outgoingState : states) {
                addTransition(incomingState, outgoingState, function.apply(outgoingState));
                addTransition(outgoingState, incomingState, function.apply(incomingState));
            }
        }
    }

    // ---- A list of (from, to) pairs ----

    /** For each (from, to) pair in {@code map}. Does nothing. */
    public void addMappedTransition(List<Pair<S, S>> map) {
        addMappedTransition(map, none());
    }

    /** For each (from, to) pair in {@code map}, running {@code runnable}. */
    public void addMappedTransition(List<Pair<S, S>> map, Runnable runnable) {
        addMappedTransition(map, runOnce(runnable));
    }

    /** For each (from, to) pair in {@code map}, running {@code command}. */
    public void addMappedTransition(List<Pair<S, S>> map, Command command) {
        map.forEach((Pair<S, S> pair) -> addTransition(pair.getFirst(), pair.getSecond(), command));
    }

    /** For each (from, to) pair in {@code map}, running the command {@code function} builds for the state being entered. */
    public void addMappedTransition(List<Pair<S, S>> map, Function<S, Command> function) {
        map.forEach((Pair<S, S> pair) -> addTransition(pair.getFirst(), pair.getSecond(), function));
    }

    // ---- A state you can enter from anywhere but only leave one way ----

    /** Lets every state go into {@code undefinedState}, which can only leave to {@code exitState}. Does nothing. */
    public void addUndefinedState(S undefinedState, S exitState)  {
        addUndefinedState(undefinedState, exitState, none());
    }

    /** Lets every state go into {@code undefinedState}, which can only leave to {@code exitState}, running {@code runnable} both ways. */
    public void addUndefinedState(S undefinedState, S exitState, Runnable runnable)  {
        addUndefinedState(undefinedState, exitState, runOnce(runnable));
    }

    /** Lets every state go into {@code undefinedState}, which can only leave to {@code exitState}, running a different runnable each way. */
    public void addUndefinedState(S undefinedState, S exitState, Runnable toUndefinedRunnable, Runnable toExitRunnable)  {
        addUndefinedState(undefinedState, exitState, runOnce(toUndefinedRunnable), runOnce(toExitRunnable));
    }

    /** Lets every state go into {@code undefinedState}, which can only leave to {@code exitState}, running {@code command} both ways. */
    public void addUndefinedState(S undefinedState, S exitState, Command command)  {
        addUndefinedState(undefinedState, exitState, command, command);
    }

    /**
     * Lets every state go into {@code undefinedState}, e.g. a manual override, which can only leave
     * to {@code exitState}, e.g. NEUTRAL.
     *
     * @param toUndefinedCommand Runs when entering {@code undefinedState}
     * @param toExitCommand Runs when leaving to {@code exitState}
     */
    public void addUndefinedState(S undefinedState, S exitState, Command toUndefinedCommand, Command toExitCommand) {
        addConvergingTransition(undefinedState, toUndefinedCommand);
        addTransition(undefinedState, exitState, toExitCommand);
    }

    /** Lets every state go into {@code undefinedState}, which can only leave to {@code exitState}, running the command {@code function} builds for the state being entered. */
    public void addUndefinedState(S undefinedState, S exitState, Function<S, Command> function) {
        addConvergingTransition(undefinedState, function);
        addTransition(undefinedState, exitState, function);
    }



    // ---- Looking up and removing transitions ----

    @SuppressWarnings("unchecked")
    private Transition<S> getAsTransition(int x, int y) {
        return (Transition<S>) adjacencyMap[x][y];
    }

    /** The state with this position in the enum, e.g. 0 for the first. */
    public S fromOrdinal(int ordinal) {
        return states.get(ordinal);
    }

    /** Same as {@link #addTransition(Transition)}. */
    public void setTransition(Transition<S> transition) {
        if(transition.getOutgoingState().ordinal() == transition.getIncomingState().ordinal()) return;
        adjacencyMap[transition.getOutgoingState().ordinal()][transition.getIncomingState().ordinal()] = transition;
    }

    /** Disallows the move from {@code start} to {@code end}. */
    public void removeTransition(S start, S end) {
        adjacencyMap[start.ordinal()][end.ordinal()] = null;
    }

    /** Disallows every move out of {@code start}. */
    public void removeAllTransitionsFromState(S start) {
        for (S s : states) {
          removeTransition(start, s);
        }
    }

    /** Disallows every move into {@code end}. */
    public void removeAllTransitionsToState(S end) {
        for (S s : states) {
          removeTransition(s, end);
        }
    }

    /** Empties the map. */
    public void removeAllTransitions() {
        for (S s1 : states) {
            for (S s2 : states) {
                removeTransition(s1, s2);
            }
        }
    }

    /** The transition from {@code start} to {@code end}; null if that move isn't allowed. */
    public Transition<S> getTransition(S start, S end) {
        return getAsTransition(start.ordinal(), end.ordinal());
    }

    /** Every transition out of {@code state}. */
    public List<Transition<S>> getOutgoingTransitions(S state) {
        List<Transition<S>> outgoing = new ArrayList<>();
        for (int i = 0; i < adjacencyMap.length; i++) {
            Transition<S> out = getAsTransition(state.ordinal(), i);
            if (out != null) outgoing.add(out);
        }
        return outgoing;
    }

    /** Every transition into {@code state}. */
    public List<Transition<S>> getIncomingTransitions(S state) {
        List<Transition<S>> incoming = new ArrayList<>();
        for (int i = 0; i < adjacencyMap.length; i++) {
            Transition<S> in = getAsTransition(i, state.ordinal());
            if (in != null) incoming.add(in);
        }
        return incoming;
    }

    /** The whole map as a grid, one row per state it leaves, for debugging. */
    @Override
    public String toString() {
        String out = "Adjacency Map " + enumType.getCanonicalName();
        for(Object[] row : adjacencyMap) {
            out = out + "\n";
            out = out + Arrays.toString(row);
        }
        return out;
    }
}
