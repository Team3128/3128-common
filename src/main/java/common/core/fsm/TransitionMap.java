package common.core.fsm;
import java.util.*;
import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * A graph that represents transitions between states of an enum.
 * 
 * @author ShamLib by team 5907
 *
 * @param <S> The enum type that represents the states of the graph
 */
public class TransitionMap<S extends Enum<S>> {

    public enum TransitionType {
        Incoming,
        Outgoing,
        All
    }

    // Array where the index [1][2] = TransitionBase from state at ordinal 1 to state at ordinal 2
    private final Object[][] adjacencyMap;
    @SuppressWarnings("unused")
    private final Class<S> enumType;
    private final S[] states;

    public TransitionMap(Class<S> enumType) {
        this.enumType = enumType;
        this.states = enumType.getEnumConstants();
        

        adjacencyMap = new Object[states.length][states.length];
    }

    /**
     * Update the adjacency map to include the data for the given transition Note: This WILL NOT
     * overwrite an existing Transition
     *
     * @param transition The transition to add to the graph
     */
    public void addTransition(Transition<S> transition) {
        setTransition(transition);
    }

    @SuppressWarnings("unchecked")
    public void addTransitions(Transition<S>... transitions) {
        for (Transition<S> transition : transitions) {
            addTransition(transition);
        }
    }

    public void addTransition(S start, S end, Command command) {
        addTransition(new Transition<S>(start, end, command));
    }

    /**
     * Adds a transition from every state to the given state
     *
     * @param state the state to go to
     */
    public void addConvergingTransition(S state, Command command) {
        for (S s : states) {
            if (s != state) {
                addTransition(s, state, command);
            }
        }
    }

    /**
     * Adds a transition from every state to the given state
     *
     * @param state the state to go to
     * @param run the transition runnable to run as an instant command
     */
    public final void addConvergingTransition(S state, Runnable run) {
        addConvergingTransition(state, new InstantCommand(run));
    }

    public final void addConvergingTransition(S state) {
        addConvergingTransition(state, () -> {});
    }

    @SuppressWarnings("unchecked")
    /**
     * Link multiple states to a single state using the same transition command
     * @param command Transition command
     * @param incomingState End state of all the transitions
     * @param outGoingStates Starting states of the each of the transitions
     */
    public void addConvergingTransitions(Command command, S incomingState, S... outGoingStates) {
        for (S outGoingState : outGoingStates) {
            addTransition(outGoingState, incomingState, command);
        }
    }

    @SuppressWarnings("unchecked")
    /**
     * Link multiple states to a single state using the same transition command
     * @param commandFunction Transition command that takes in each outgoing state
     * @param incomingState End state of all the transitions
     * @param outGoingStates Starting states of the each of the transitions
     */
    public void addConvergingFunctions(Function<S, Command> commandFunction, S incomingState, S... outGoingStates) {
        for (S outGoingState : outGoingStates) {
            addTransition(outGoingState, incomingState, commandFunction.apply(outGoingState));
        }
    }

    @SuppressWarnings("unchecked")
    public void addConvergingFunctions(Function<S, Command> function, S... states) {
        for (S s1 : states) {
            Command result = function.apply(s1);
            if (result != null) {
                addConvergingTransition(s1, result);
            }
        }
    }

    public void addConvergingFunctions(Function<S, Command> function) {
        addConvergingFunctions(function, states);
    }

    /**
     * Adds a transition from the given state to every other state
     *
     * @param state the state to go from
     * @param command transition command to run
     */
    public void addDivergingTransition(S state, Command command) {
        for (S s : states) {
            if (s != state) {
                addTransition(state, s, command);
            }
        }
    }

    /**
     * Adds a transition from every state to the given state
     *
     * @param state the state to go to
     * @param run the transition runnable to run as an instant command
     */
    public void addDivergingTransition(S state, Runnable run) {
        addDivergingTransition(state, new InstantCommand(run));
    }

    public void addDivergingTransition(S state) {
        addDivergingTransition(state, () -> {});
    }

    @SuppressWarnings("unchecked")
    public void addDivergingTransitions(S... states) {
        for (S state : states) {
            addDivergingTransition(state);
        }
    }

    
    /**
     * Adds a transition from the given states to every other state
     * @param function function to apply to each incoming state
     * @param state states to diverge from
     */
    public void applyDivergingFunction(Function<S, Command> function, S state) {
        applyDivergingFunction(function, state, states);
    }

    @SuppressWarnings("unchecked")
    public void applyDivergingFunction(Function<S, Command> function, S divergentState, S... divergeToStates) {
        for(S divergeToState : divergeToStates) {
            Command result = function.apply(divergentState);
            if (result != null) {
                addTransition(divergentState, divergeToState, result);
            }
        }
    }

    /**
     * Add a transition both ways between two states
     *
     * @param start beginning state
     * @param end ending state
     * @param run the command to run between them
     */
    public void addCommutativeTransition(S start, S end, Command run) {
        addTransition(new Transition<>(start, end, run));
        addTransition(new Transition<>(end, start, run));
    }

    /**
     * Add a transition both ways between two states
     *
     * @param start beginning state
     * @param end ending state
     * @param toRun the runnable to run between them
     */
    public void addCommutativeTransition(S start, S end, Runnable toRun) {
        addCommutativeTransition(start, end, Commands.runOnce(toRun));
    }

    /**
     * Add a transition both ways between two states
     *
     * @param start beginning state
     * @param end ending state
     */
    public void addCommutativeTransition(S start, S end) {
        addCommutativeTransition(start, end, Commands.none());
    }

    /**
     * Add a transition both ways between two states
     *
     * @param start beginning state
     * @param end ending state
     * @param run1 the command to run between start and end
     * @param run2 the command to run between end and start
     */
    public void addCommutativeTransition(S start, S end, Command run1, Command run2) {
        addTransition(start, end, run1);
        addTransition(end, start, run2);
    }

    /**
     * Add a transition both ways between two states
     *
     * @param start beginning state
     * @param end ending state
     * @param run1 the runnable to run between start and end
     * @param run2 the runnable to run between end and start
     */
    public void addCommutativeTransition(S start, S end, Runnable run1, Runnable run2) {
        addCommutativeTransition(start, end, Commands.runOnce(run1), Commands.runOnce(run2));
    }

    public void applyCommutativeFunction(Function<S, Command> function, S... states) {
        for(S state1 : states) {
            for(S state2 : states) {
                addTransition(state1, state2, function.apply(state2));
                addTransition(state2, state1, function.apply(state1));
            }
        }
    }

    @SuppressWarnings("unchecked")
    private Transition<S> getAsTransition(int x, int y) {
        return (Transition<S>) adjacencyMap[x][y];
    }

    @SuppressWarnings("unused")
    private S fromOrdinal(int ordinal) {
        return states[ordinal];
    }

    /**
     * Update the adjacency map to include the data for the given transition. Note: This WILL override
     * an existing transition, if one is present
     *
     * @param transition The transition to set to the graph
     */
    public void setTransition(Transition<S> transition) {
        if(transition.getOutgoingState().ordinal() == transition.getIncomingState().ordinal()) return;
        adjacencyMap[transition.getOutgoingState().ordinal()][transition.getIncomingState().ordinal()] = transition;
    }

    public void removeTransition(S start, S end) {
        adjacencyMap[start.ordinal()][end.ordinal()] = null;
    }

    public void removeAllTransitionsFromState(S start) {
        for (S s : states) {
          removeTransition(start, s);
        }
    }

    public void removeAllTransitionsToState(S end) {
        for (S s : states) {
          removeTransition(s, end);
        }
    }

    public void removeAllTransitions() {
        for (S s1 : states) {
            for (S s2 : states) {
                removeTransition(s1, s2);
            }
        }
    }

    /**
     * Returns the Transition of the graph
     *
     * @param start starting State of the Transition
     * @param end ending State of the Transition
     * @return the Transition, if one is found. Otherwise, the method will return null
     */
    public Transition<S> getTransition(S start, S end) {
        return getAsTransition(start.ordinal(), end.ordinal());
    }

    public List<Transition<S>> getOutgoingTransitions(S state) {
        List<Transition<S>> outgoing = new ArrayList<>();
        for (int i = 0; i < adjacencyMap.length; i++) {
            Transition<S> out = getAsTransition(state.ordinal(), i);
            if (out != null) outgoing.add(out);
        }
        return outgoing;
    }

    public List<Transition<S>> getIncomingTransitions(S state) {
        List<Transition<S>> incoming = new ArrayList<>();
        for (int i = 0; i < adjacencyMap.length; i++) {
            Transition<S> in = getAsTransition(i, state.ordinal());
            if (in != null) incoming.add(in);
        }
        return incoming;
    }

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
