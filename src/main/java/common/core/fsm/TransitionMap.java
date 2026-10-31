package common.core.fsm;
import java.util.*;
import java.util.function.Function;

import common.utility.Log;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

/**
 * The allowed moves between the states of an {@link FSMSubsystemBase}, and the command each one
 * runs. A move that is not in the map is not allowed.
 *
 * <p>Adding a transition between two states that already have one replaces it. Transitions from a
 * state to itself are ignored. See {@code README.md} in this package for the shortcut methods.
 *
 * <p>Based on ShamLib by team 5907.
 *
 * @author Teja Yaramada
 *
 * @param <S> The enum listing the states
 */
public class TransitionMap<S extends Enum<S>> {

    /** {@code adjacencyMap[from][to]} is the transition between those state ordinals, or null. */
    private final Object[][] adjacencyMap;
    @SuppressWarnings("unused")
    private final Class<S> enumType;
    private final List<S> states;

    /** @param enumType The enum class of the states, for example, {@code IntakeStates.class} */
    public TransitionMap(Class<S> enumType) {
        this.enumType = enumType;
        this.states = List.of(enumType.getEnumConstants());


        adjacencyMap = new Object[states.size()][states.size()];
    }

    /**
     * @return Whether no transitions have been added
     */
    public boolean isEmpty() {
        return getTransitionCount() == 0;
    }

    /**
     * Returns the number of transitions in the map. The count is recalculated on each call, so adding
     * a transition twice or removing a transition that does not exist cannot make it inaccurate.
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

    /** Adds a transition from {@code start} to {@code end}, running {@code runnable} once. */
    public void addTransition(S start, S end, Runnable runnable) {
        addTransition(start, end, runOnce(runnable));
    }

    /** Adds a transition from {@code start} to {@code end}, running {@code command}. */
    public void addTransition(S start, S end, Command command) {
        addTransition(new Transition<S>(start, end, command));
    }

    /** Adds a transition from {@code start} to {@code end}, running the command that {@code function} builds for {@code end}. */
    public void addTransition(S start, S end, Function<S, Command> function){
        addTransition(start, end, function.apply(end));
    }

    /** Adds transitions from each state in {@code start} to the state at the same position in {@code end}. These transitions run no command. */
    public void addCorrespondenceTransitions(List<S> start, List<S> end) {
        addCorrespondenceTransitions(start, end, none());
    }

    /** Adds transitions from each state in {@code start} to the state at the same position in {@code end}, running {@code runnable} once. */
    public void addCorrespondenceTransitions(List<S> start, List<S> end, Runnable runnable) {
        addCorrespondenceTransitions(start, end, runOnce(runnable));
    }

    /** Adds transitions from each state in {@code start} to the state at the same position in {@code end}, running {@code command}. */
    public void addCorrespondenceTransitions(List<S> start, List<S> end, Command command) {
        addCorrespondenceTransitions(start, end, state -> {return command;});
    }

    /**
     * Adds transitions from each state in {@code start} to the state at the same position in {@code end}, running the
     * command {@code function} builds for the state being entered. The lists must be the same length.
     */
    public void addCorrespondenceTransitions(List<S> start, List<S> end, Function<S, Command> function) {
        if(start.size() != end.size()) Log.fatal("Transition Map", "Failed to load one to one transition");
        for(int i = 0; i < end.size(); i++) {
            addTransition(start.get(i), end.get(i), function);
        }
    }

    /** Adds transitions from every state into {@code end}. These transitions run no command. */
    public void addConvergingTransition(S end) {
        addConvergingTransition(end, none());
    }

    /** Adds transitions from every state into {@code end}, running {@code runnable} once. */
    public void addConvergingTransition(S end, Runnable runnable) {
        addConvergingTransition(end, runOnce(runnable));
    }

    /** Adds transitions from every state into {@code end}, running {@code command}. */
    public void addConvergingTransition(S end, Command command) {
        for (S start : states) {
            addTransition(start, end, command);
        }
    }

    /** Adds transitions from every state into {@code end}, running the command that {@code function} builds for {@code end}. */
    public void addConvergingTransition(S end, Function<S, Command> function) {
        addConvergingTransition(end, function.apply(end));
    }

    /** Adds transitions from each state in {@code start} into {@code end}. These transitions run no command. */
    public void addConvergingTransition(List<S> start, S end) {
        addConvergingTransition(start, end, none());
    }

    /** Adds transitions from each state in {@code start} into {@code end}, running {@code runnable} once. */
    public void addConvergingTransition(List<S> start, S end, Runnable runnable) {
        addConvergingTransition(start, end, runOnce(runnable));
    }

    /** Adds transitions from each state in {@code start} into {@code end}, running {@code command}. */
    public void addConvergingTransition(List<S> start, S end, Command command) {
        for (S outGoingState : start) {
            addTransition(outGoingState, end, command);
        }
    }

    /** Adds transitions from each state in {@code start} into {@code end}, running the command that {@code function} builds for {@code end}. */
    public void addConvergingTransition(List<S> start, S end, Function<S, Command> function) {
        addConvergingTransition(start, end, function.apply(end));
    }

    /** Adds transitions from every state into each state in {@code end}. These transitions run no command. */
    public void addConvergingTransition(List<S> end) {
        addConvergingTransition(end, none());
    }

    /** Adds transitions from every state into each state in {@code end}, running {@code runnable} once. */
    public void addConvergingTransition(List<S> end, Runnable runnable) {
        addConvergingTransition(end, runOnce(runnable));
    }

    /** Adds transitions from every state into each state in {@code end}, running {@code command}. */
    public void addConvergingTransition(List<S> end, Command command) {
        for(S incomingState : end) {
            addConvergingTransition(incomingState, command);
        }
    }

    /** Adds transitions from every state into each state in {@code end}, running the command that {@code function} builds for the state being entered. */
    public void addConvergingTransition(List<S> end, Function<S, Command> function) {
        for (S incomingState : end) {
           addConvergingTransition(incomingState, function.apply(incomingState));
        }
    }

    /** Adds transitions from each state in {@code start} into each state in {@code end}. These transitions run no command. */
    public void addConvergingTransition(List<S> start, List<S> end) {
        addConvergingTransition(start, end, none());
    }

    /** Adds transitions from each state in {@code start} into each state in {@code end}, running {@code runnable} once. */
    public void addConvergingTransition(List<S> start, List<S> end, Runnable runnable) {
        addConvergingTransition(start, end, runOnce(runnable));
    }

    /** Adds transitions from each state in {@code start} into each state in {@code end}, running {@code command}. */
    public void addConvergingTransition(List<S> start, List<S> end, Command command) {
        for(S incomingState : end) {
            addConvergingTransition(start, incomingState, command);
        }
    }

    /** Adds transitions from each state in {@code start} into each state in {@code end}, running the command that {@code function} builds for the state being entered. */
    public void addConvergingTransition(List<S> start, List<S> end, Function<S, Command> function) {
        for(S incomingState : end) {
            addConvergingTransition(start, incomingState, function.apply(incomingState));
        }
    }

    /** Adds transitions from {@code start} to every other state. These transitions run no command. */
    public void addDivergingTransition(S start) {
        addDivergingTransition(start, none());
    }

    /** Adds transitions from {@code start} to every other state, running {@code runnable} once. */
    public void addDivergingTransition(S start, Runnable runnable) {
        addDivergingTransition(start, runOnce(runnable));
    }

    /** Adds transitions from {@code start} to every other state, running {@code command}. */
    public void addDivergingTransition(S start, Command command) {
        addDivergingTransition(start, states, command);
    }

    /** Adds transitions from {@code start} to every other state, running the command that {@code function} builds for the state being entered. */
    public void addDivergingTransition(S start, Function<S, Command> function) {
        addDivergingTransition(start, states, function);
    }

    /** Adds transitions from {@code start} to each state in {@code end}. These transitions run no command. */
    public void addDivergingTransition(S start, List<S> end) {
        addDivergingTransition(start, end, none());
    }

    /** Adds transitions from {@code start} to each state in {@code end}, running {@code runnable} once. */
    public void addDivergingTransition(S start, List<S> end, Runnable runnable) {
        addDivergingTransition(start, end, runOnce(runnable));
    }

    /** Adds transitions from {@code start} to each state in {@code end}, running {@code command}. */
    public void addDivergingTransition(S start, List<S> end, Command command) {
        for (S incomingState : end) {
            addTransition(start, incomingState, command);
        }
    }

    /** Adds transitions from {@code start} to each state in {@code end}, running the command that {@code function} builds for the state being entered. */
    public void addDivergingTransition(S start, List<S> end, Function<S, Command> function) {
        for (S incomingState : end) {
            addTransition(start, incomingState, function.apply(incomingState));
        }
    }

    /** Adds transitions from each state in {@code start} to every other state. These transitions run no command. */
    public void addDivergingTransition(List<S> start) {
        addDivergingTransition(start, none());
    }

    /** Adds transitions from each state in {@code start} to every other state, running {@code runnable} once. */
    public void addDivergingTransition(List<S> start, Runnable runnable) {
        addDivergingTransition(start, runOnce(runnable));
    }

    /** Adds transitions from each state in {@code start} to every other state, running {@code command}. */
    public void addDivergingTransition(List<S> start, Command command) {
        for (S outgoingState : start) {
            addDivergingTransition(outgoingState, command);
        }
    }

    /** Adds transitions from each state in {@code start} to every other state, running the command that {@code function} builds for the state being entered. */
    public void addDivergingTransition(List<S> start, Function<S, Command> function) {
        for (S outgoingState : start) {
            addDivergingTransition(outgoingState, function);
        }
    }

    /** Adds transitions from each state in {@code start} to each state in {@code end}. These transitions run no command. */
    public void addDivergingTransition(List<S> start, List<S> end) {
        addDivergingTransition(start, end, none());
    }

    /** Adds transitions from each state in {@code start} to each state in {@code end}, running {@code runnable} once. */
    public void addDivergingTransition(List<S> start, List<S> end, Runnable runnable) {
        addDivergingTransition(start, end, runOnce(runnable));
    }

    /** Adds transitions from each state in {@code start} to each state in {@code end}, running {@code command}. */
    public void addDivergingTransition(List<S> start, List<S> end, Command command) {
        for (S outgoingState : start) {
            addDivergingTransition(outgoingState, end, command);
        }
    }

    /** Adds transitions from each state in {@code start} to each state in {@code end}, running the command that {@code function} builds for the state being entered. */
    public void addDivergingTransition(List<S> start, List<S> end, Function<S, Command> function) {
        for (S outgoingState : start) {
            addDivergingTransition(outgoingState, end, function);
        }
    }

    /** Adds transitions between {@code start} and {@code end} in both directions. These transitions run no command. */
    public void addCommutativeTransition(S start, S end) {
        addCommutativeTransition(start, end, none());
    }

    /** Adds transitions between {@code start} and {@code end} in both directions, running {@code runnable} once. */
    public void addCommutativeTransition(S start, S end, Runnable runnable) {
        addCommutativeTransition(start, end, runOnce(runnable));
    }

    /** Adds transitions between {@code start} and {@code end}: {@code runnableForwards} going to {@code end}, {@code runnableBackwards} coming back. */
    public void addCommutativeTransition(S start, S end, Runnable runnableForwards, Runnable runnableBackwards) {
        addCommutativeTransition(start, end, runOnce(runnableForwards), runOnce(runnableBackwards));
    }

    /** Adds transitions between {@code start} and {@code end} in both directions, running {@code command}. */
    public void addCommutativeTransition(S start, S end, Command command) {
        addCommutativeTransition(start, end, command, command);
    }

    /** Adds transitions between {@code start} and {@code end}: {@code commandForwards} going to {@code end}, {@code commandBackwards} coming back. */
    public void addCommutativeTransition(S start, S end, Command commandForwards, Command commandBackwards) {
        addTransition(start, end, commandForwards);
        addTransition(end, start, commandBackwards);
    }

    /** Adds transitions between {@code start} and {@code end} in both directions, each running the command that {@code function} builds for the state being entered. */
    public void addCommutativeTransition(S start, S end, Function<S, Command> function) {
        addTransition(start, end, function);
        addTransition(end, start, function);
    }

    /** Adds transitions between each state in {@code start} and {@code end} in both directions. These transitions run no command. */
    public void addCommutativeTransition(List<S> start, S end) {
        addCommutativeTransition(start, end, none());
    }

    /** Adds transitions between each state in {@code start} and {@code end} in both directions, running {@code runnable} once. */
    public void addCommutativeTransition(List<S> start, S end, Runnable runnable) {
        addCommutativeTransition(start, end, runOnce(runnable));
    }

    /** Adds transitions between each state in {@code start} and {@code end}: {@code runnableForwards} going to {@code end}, {@code runnableBackwards} coming back. */
    public void addCommutativeTransition(List<S> start, S end, Runnable runnableForwards, Runnable runnableBackwards) {
        addCommutativeTransition(start, end, runOnce(runnableForwards), runOnce(runnableBackwards));
    }

    /** Adds transitions between each state in {@code start} and {@code end} in both directions, running {@code command}. */
    public void addCommutativeTransition(List<S> start, S end, Command command) {
        addCommutativeTransition(start, end, command, command);
    }

    /** Adds transitions between each state in {@code start} and {@code end}: {@code commandForwards} going to {@code end}, {@code commandBackwards} coming back. */
    public void addCommutativeTransition(List<S> start, S end, Command commandForwards, Command commandBackwards) {
        for (S outgoingState : start) {
            addCommutativeTransition(outgoingState, end, commandForwards, commandBackwards);
        }
    }

    /**
     * Adds transitions between each state in {@code start} and {@code end} in both directions, each running the command that
     * {@code function} builds for the state being entered.
     */
    public void addCommutativeTransition(List<S> start, S end, Function<S, Command> function) {
        for (S outgoingState : start) {
            addCommutativeTransition(outgoingState, end, function);
        }
    }

    /** Adds transitions between {@code start} and each state in {@code end} in both directions. These transitions run no command. */
    public void addCommutativeTransition(S start, List<S> end) {
        addCommutativeTransition(start, end, none());
    }

    /** Adds transitions between {@code start} and each state in {@code end} in both directions, running {@code runnable} once. */
    public void addCommutativeTransition(S start, List<S> end, Runnable runnable) {
        addCommutativeTransition(start, end, runOnce(runnable));
    }

    /** Adds transitions between {@code start} and each state in {@code end}: {@code runnableForwards} going out, {@code runnableBackwards} coming back. */
    public void addCommutativeTransition(S start, List<S> end, Runnable runnableForwards, Runnable runnableBackwards) {
        addCommutativeTransition(start, end, runOnce(runnableForwards), runOnce(runnableBackwards));
    }

    /** Adds transitions between {@code start} and each state in {@code end} in both directions, running {@code command}. */
    public void addCommutativeTransition(S start, List<S> end, Command command) {
        addCommutativeTransition(start, end, command, command);
    }

    /** Adds transitions between {@code start} and each state in {@code end}: {@code commandForwards} going out, {@code commandBackwards} coming back. */
    public void addCommutativeTransition(S start, List<S> end, Command commandForwards, Command commandBackwards) {
        for (S incomingState : end) {
            addCommutativeTransition(start, incomingState, commandForwards, commandBackwards);
        }
    }

    /**
     * Adds transitions between {@code start} and each state in {@code end} in both directions, each running the command that
     * {@code function} builds for the state being entered.
     */
    public void addCommutativeTransition(S start, List<S> end, Function<S, Command> function) {
        for (S incomingState : end) {
            addCommutativeTransition(start, incomingState, function);
        }
    }

    /** Adds transitions between every pair of states in {@code states} in both directions. These transitions run no command. */
    public void addCommutativeTransition(List<S> states) {
        addCommutativeTransition(states, none());
    }

    /** Adds transitions between every pair of states in {@code states} in both directions, running {@code runnable} once. */
    public void addCommutativeTransition(List<S> states, Runnable runnable) {
        addCommutativeTransition(states, runOnce(runnable));
    }

    /** Adds transitions between every pair of states in {@code states} in both directions, running {@code command}. */
    public void addCommutativeTransition(List<S> states, Command command) {
        addCommutativeTransition(states, state -> {return command;});
    }

    /**
     * Adds transitions between every pair of states in {@code states} in both directions, each running the command that
     * {@code function} builds for the state being entered. Useful when every state has its own
     * setpoints, for example, {@code addCommutativeTransition(List.of(States.values()), state -> ...)}.
     */
    public void addCommutativeTransition(List<S> states, Function<S, Command> function) {
        for(S incomingState : states) {
            for(S outgoingState : states) {
                addTransition(incomingState, outgoingState, function.apply(outgoingState));
                addTransition(outgoingState, incomingState, function.apply(incomingState));
            }
        }
    }

    /** Adds a transition for each (from, to) pair in {@code map}. These transitions run no command. */
    public void addMappedTransition(List<Pair<S, S>> map) {
        addMappedTransition(map, none());
    }

    /** Adds a transition for each (from, to) pair in {@code map}, running {@code runnable} once. */
    public void addMappedTransition(List<Pair<S, S>> map, Runnable runnable) {
        addMappedTransition(map, runOnce(runnable));
    }

    /** Adds a transition for each (from, to) pair in {@code map}, running {@code command}. */
    public void addMappedTransition(List<Pair<S, S>> map, Command command) {
        map.forEach((Pair<S, S> pair) -> addTransition(pair.getFirst(), pair.getSecond(), command));
    }

    /** Adds a transition for each (from, to) pair in {@code map}, running the command that {@code function} builds for the state being entered. */
    public void addMappedTransition(List<Pair<S, S>> map, Function<S, Command> function) {
        map.forEach((Pair<S, S> pair) -> addTransition(pair.getFirst(), pair.getSecond(), function));
    }

    /** Allows every state to enter {@code undefinedState}, which may exit only to {@code exitState}. These transitions run no command. */
    public void addUndefinedState(S undefinedState, S exitState)  {
        addUndefinedState(undefinedState, exitState, none());
    }

    /** Allows every state to enter {@code undefinedState}, which may exit only to {@code exitState}, running {@code runnable} once in both directions. */
    public void addUndefinedState(S undefinedState, S exitState, Runnable runnable)  {
        addUndefinedState(undefinedState, exitState, runOnce(runnable));
    }

    /** Allows every state to enter {@code undefinedState}, which may exit only to {@code exitState}, running a different runnable each way. */
    public void addUndefinedState(S undefinedState, S exitState, Runnable toUndefinedRunnable, Runnable toExitRunnable)  {
        addUndefinedState(undefinedState, exitState, runOnce(toUndefinedRunnable), runOnce(toExitRunnable));
    }

    /** Allows every state to enter {@code undefinedState}, which may exit only to {@code exitState}, running {@code command} in both directions. */
    public void addUndefinedState(S undefinedState, S exitState, Command command)  {
        addUndefinedState(undefinedState, exitState, command, command);
    }

    /**
     * Allows every state to enter {@code undefinedState}, for example, a manual override, which may
     * exit only to {@code exitState}, for example, NEUTRAL.
     *
     * @param toUndefinedCommand The command that runs when entering {@code undefinedState}
     * @param toExitCommand The command that runs when exiting to {@code exitState}
     */
    public void addUndefinedState(S undefinedState, S exitState, Command toUndefinedCommand, Command toExitCommand) {
        addConvergingTransition(undefinedState, toUndefinedCommand);
        addTransition(undefinedState, exitState, toExitCommand);
    }

    /** Allows every state to enter {@code undefinedState}, which may exit only to {@code exitState}, running the command that {@code function} builds for the state being entered. */
    public void addUndefinedState(S undefinedState, S exitState, Function<S, Command> function) {
        addConvergingTransition(undefinedState, function);
        addTransition(undefinedState, exitState, function);
    }

    @SuppressWarnings("unchecked")
    private Transition<S> getAsTransition(int x, int y) {
        return (Transition<S>) adjacencyMap[x][y];
    }

    /** Returns the state at this position in the enum, for example, 0 for the first. */
    public S fromOrdinal(int ordinal) {
        return states.get(ordinal);
    }

    /** Equivalent to {@link #addTransition(Transition)}. */
    public void setTransition(Transition<S> transition) {
        if(transition.getOutgoingState().ordinal() == transition.getIncomingState().ordinal()) return;
        adjacencyMap[transition.getOutgoingState().ordinal()][transition.getIncomingState().ordinal()] = transition;
    }

    /** Removes the transition from {@code start} to {@code end}. */
    public void removeTransition(S start, S end) {
        adjacencyMap[start.ordinal()][end.ordinal()] = null;
    }

    /** Removes every transition out of {@code start}. */
    public void removeAllTransitionsFromState(S start) {
        for (S s : states) {
          removeTransition(start, s);
        }
    }

    /** Removes every transition into {@code end}. */
    public void removeAllTransitionsToState(S end) {
        for (S s : states) {
          removeTransition(s, end);
        }
    }

    /** Removes every transition from the map. */
    public void removeAllTransitions() {
        for (S s1 : states) {
            for (S s2 : states) {
                removeTransition(s1, s2);
            }
        }
    }

    /** Returns the transition from {@code start} to {@code end}, or null if that transition is not allowed. */
    public Transition<S> getTransition(S start, S end) {
        return getAsTransition(start.ordinal(), end.ordinal());
    }

    /** Returns every transition out of {@code state}. */
    public List<Transition<S>> getOutgoingTransitions(S state) {
        List<Transition<S>> outgoing = new ArrayList<>();
        for (int i = 0; i < adjacencyMap.length; i++) {
            Transition<S> out = getAsTransition(state.ordinal(), i);
            if (out != null) outgoing.add(out);
        }
        return outgoing;
    }

    /** Returns every transition into {@code state}. */
    public List<Transition<S>> getIncomingTransitions(S state) {
        List<Transition<S>> incoming = new ArrayList<>();
        for (int i = 0; i < adjacencyMap.length; i++) {
            Transition<S> in = getAsTransition(i, state.ordinal());
            if (in != null) incoming.add(in);
        }
        return incoming;
    }

    /** Returns the whole map as a grid with one row per outgoing state, for debugging. */
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
