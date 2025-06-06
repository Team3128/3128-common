package common.core.fsm;
import java.util.*;
import java.util.function.Function;

import common.utility.Log;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

/**
 * A graph that represents transitions between states of an enum.
 * 
 * Based on ShamLib by team 5907
 * 
 * @author Teja Yaramada
 *
 * @param <S> The enum type that represents the states of the graph
 */
public class TransitionMap<S extends Enum<S>> {

    // Array where the index [1][2] = Transition from state at ordinal 1 to state at ordinal 2
    private final Object[][] adjacencyMap;
    @SuppressWarnings("unused")
    private final Class<S> enumType;
    private final List<S> states;
    private boolean isEmpty;
    private int transitionCount;

    public TransitionMap(Class<S> enumType) {
        this.enumType = enumType;
        this.states = List.of(enumType.getEnumConstants());
        

        adjacencyMap = new Object[states.size()][states.size()];
        isEmpty = true;
        transitionCount = 0;
    }

    /**
     * @return Whether the transition map is empty
     */
    public boolean isEmpty() {
        if(!isEmpty && transitionCount > 0) return false;
        isEmpty = true;
        transitionCount = 0;
        for(Object[] row : adjacencyMap){
            for(Object transition : row) {
                if(transition != null) {
                    isEmpty = false;
                    transitionCount++;
                }
            }
        }
        return isEmpty;
    }

    public int getTransitionCount() {
        return transitionCount;
    }

    /**
     * Update the adjacency map to include the data for the given transition Note: This WILL NOT
     * overwrite an existing Transition
     *
     * @param transition The transition to add to the graph
     */
    public void addTransition(Transition<S> transition) {
        transitionCount++;
        setTransition(transition);
    }

    @SuppressWarnings("unchecked")
    public void addTransitions(Transition<S>... transitions) {
        for (Transition<S> transition : transitions) {
            addTransition(transition);
        }
    }

    public void addTransition(S start, S end, Runnable runnable) {
        addTransition(start, end, runOnce(runnable));
    }

    public void addTransition(S start, S end, Command command) {
        addTransition(new Transition<S>(start, end, command));
    }

    public void addTransition(S start, S end, Function<S, Command> function){
        addTransition(start, end, function.apply(end));
    }



    public void addCorrespondenceTransitions(List<S> start, List<S> end) {
        addCorrespondenceTransitions(start, end, none());
    }
    
    public void addCorrespondenceTransitions(List<S> start, List<S> end, Runnable runnable) {
        addCorrespondenceTransitions(start, end, runOnce(runnable));
    }

    public void addCorrespondenceTransitions(List<S> start, List<S> end, Command command) {
        addCorrespondenceTransitions(start, end, state -> {return command;});
    }

    public void addCorrespondenceTransitions(List<S> start, List<S> end, Function<S, Command> function) {
        if(start.size() != end.size()) Log.fatal("Transition Map", "Failed to load one to one transition");
        for(int i = 0; i < end.size(); i++) {
            addTransition(start.get(i), end.get(i), function);
        }
    }



    public void addConvergingTransition(S end) {
        addConvergingTransition(end, none());
    }
    
    public void addConvergingTransition(S end, Runnable runnable) {
        addConvergingTransition(end, runOnce(runnable));
    }
    
    public void addConvergingTransition(S end, Command command) {
        for (S start : states) {
            addTransition(start, end, command);
        }
    }

    public void addConvergingTransition(S end, Function<S, Command> function) {
        addConvergingTransition(end, function.apply(end));
    }

    public void addConvergingTransition(List<S> start, S end) {
        addConvergingTransition(start, end, none());
    }

    public void addConvergingTransition(List<S> start, S end, Runnable runnable) {
        addConvergingTransition(start, end, runOnce(runnable));
    }

    public void addConvergingTransition(List<S> start, S end, Command command) {
        for (S outGoingState : start) {
            addTransition(outGoingState, end, command);
        }
    }

    public void addConvergingTransition(List<S> start, S end, Function<S, Command> function) {
        addConvergingTransition(start, end, function.apply(end));
    }

    public void addConvergingTransition(List<S> end) {
        addConvergingTransition(end, none());
    }

    public void addConvergingTransition(List<S> end, Runnable runnable) {
        addConvergingTransition(end, runOnce(runnable));
    }

    public void addConvergingTransition(List<S> end, Command command) {
        for(S incomingState : end) {
            addConvergingTransition(incomingState, command);
        }
    }

    public void addConvergingTransition(List<S> end, Function<S, Command> function) {
        for (S incomingState : end) {
           addConvergingTransition(incomingState, function.apply(incomingState));
        }
    }

    public void addConvergingTransition(List<S> start, List<S> end) {
        addConvergingTransition(start, end, none());
    }

    public void addConvergingTransition(List<S> start, List<S> end, Runnable runnable) {
        addConvergingTransition(start, end, runOnce(runnable));
    }

    public void addConvergingTransition(List<S> start, List<S> end, Command command) {
        for(S incomingState : end) {
            addConvergingTransition(start, incomingState, command);
        }
    }

    public void addConvergingTransition(List<S> start, List<S> end, Function<S, Command> function) {
        for(S incomingState : end) {
            addConvergingTransition(start, incomingState, function.apply(incomingState));
        }
    }



    public void addDivergingTransition(S start) {
        addDivergingTransition(start, none());
    }

    public void addDivergingTransition(S start, Runnable runnable) {
        addDivergingTransition(start, runOnce(runnable));
    }

    public void addDivergingTransition(S start, Command command) {
        addDivergingTransition(start, states, command);
    }

    public void addDivergingTransition(S start, Function<S, Command> function) {
        addDivergingTransition(start, states, function);
    }

    public void addDivergingTransition(S start, List<S> end) {
        addDivergingTransition(start, end, none());
    }

    public void addDivergingTransition(S start, List<S> end, Runnable runnable) {
        addDivergingTransition(start, end, runOnce(runnable));
    }

    public void addDivergingTransition(S start, List<S> end, Command command) {
        for (S incomingState : end) {
            addTransition(start, incomingState, command);
        }
    }
    
    public void addDivergingTransition(S start, List<S> end, Function<S, Command> function) {
        for (S incomingState : end) {
            addTransition(start, incomingState, function.apply(incomingState));
        }
    }

    public void addDivergingTransition(List<S> start) {
        addDivergingTransition(start, none());
    }

    public void addDivergingTransition(List<S> start, Runnable runnable) {
        addDivergingTransition(start, runOnce(runnable));
    }

    public void addDivergingTransition(List<S> start, Command command) {
        for (S outgoingState : start) {
            addDivergingTransition(outgoingState, command);
        }
    }

    public void addDivergingTransition(List<S> start, Function<S, Command> function) {
        for (S outgoingState : start) {
            addDivergingTransition(outgoingState, function);
        }
    }

    public void addDivergingTransition(List<S> start, List<S> end) {
        addDivergingTransition(start, end, none());
    }

    public void addDivergingTransition(List<S> start, List<S> end, Runnable runnable) {
        addDivergingTransition(start, end, runOnce(runnable));
    }

    public void addDivergingTransition(List<S> start, List<S> end, Command command) {
        for (S outgoingState : start) {
            addDivergingTransition(outgoingState, end, command);
        }
    }

    public void addDivergingTransition(List<S> start, List<S> end, Function<S, Command> function) {
        for (S outgoingState : start) {
            addDivergingTransition(outgoingState, end, function);
        }
    }



    public void addCommutativeTransition(S start, S end) {
        addCommutativeTransition(start, end, none());
    }

    public void addCommutativeTransition(S start, S end, Runnable runnable) {
        addCommutativeTransition(start, end, runOnce(runnable));
    }

    public void addCommutativeTransition(S start, S end, Runnable runnableForwards, Runnable runnableBackwards) {
        addCommutativeTransition(start, end, runOnce(runnableForwards), runOnce(runnableBackwards));
    }

    public void addCommutativeTransition(S start, S end, Command command) {
        addCommutativeTransition(start, end, command, command);
    }

    public void addCommutativeTransition(S start, S end, Command commandForwards, Command commandBackwards) {
        addTransition(start, end, commandForwards);
        addTransition(end, start, commandBackwards);
    }

    public void addCommutativeTransition(S start, S end, Function<S, Command> function) {
        addTransition(start, end, function);
        addTransition(end, start, function);
    }

    public void addCommutativeTransition(List<S> start, S end) {
        addCommutativeTransition(start, end, none());
    }

    public void addCommutativeTransition(List<S> start, S end, Runnable runnable) {
        addCommutativeTransition(start, end, runOnce(runnable));
    }

    public void addCommutativeTransition(List<S> start, S end, Runnable runnableForwards, Runnable runnableBackwards) {
        addCommutativeTransition(start, end, runOnce(runnableForwards), runOnce(runnableBackwards));
    }

    public void addCommutativeTransition(List<S> start, S end, Command command) {
        addCommutativeTransition(start, end, command, command);
    }

    public void addCommutativeTransition(List<S> start, S end, Command commandForwards, Command commandBackwards) {
        for (S outgoingState : start) {
            addCommutativeTransition(outgoingState, end, commandForwards, commandBackwards);
        }
    }

    public void addCommutativeTransition(List<S> start, S end, Function<S, Command> function) {
        addCommutativeTransition(start, end, function.apply(end));
    }

    public void addCommutativeTransition(S start, List<S> end) {
        addCommutativeTransition(start, end, none());
    }

    public void addCommutativeTransition(S start, List<S> end, Runnable runnable) {
        addCommutativeTransition(start, end, runOnce(runnable));
    }

    public void addCommutativeTransition(S start, List<S> end, Runnable runnableForwards, Runnable runnableBackwards) {
        addCommutativeTransition(start, end, runOnce(runnableForwards), runOnce(runnableBackwards));
    }

    public void addCommutativeTransition(S start, List<S> end, Command command) {
        addCommutativeTransition(start, end, command, command);
    }

    public void addCommutativeTransition(S start, List<S> end, Command commandForwards, Command commandBackwards) {
        for (S incomingState : end) {
            addCommutativeTransition(start, incomingState, commandForwards, commandBackwards);
        }
    }

    public void addCommutativeTransition(S start, List<S> end, Function<S, Command> function) {
        for (S incomingState : end) {
            addCommutativeTransition(start, incomingState, function.apply(incomingState));
        }
    }

    public void addCommutativeTransition(List<S> states) {
        addCommutativeTransition(states, none());
    }

    public void addCommutativeTransition(List<S> states, Runnable runnable) {
        addCommutativeTransition(states, runOnce(runnable));
    }

    public void addCommutativeTransition(List<S> states, Command command) {
        addCommutativeTransition(states, state -> {return command;});
    }

    public void addCommutativeTransition(List<S> states, Function<S, Command> function) {
        for(S incomingState : states) {
            for(S outgoingState : states) {
                addTransition(incomingState, outgoingState, function.apply(outgoingState));
                addTransition(outgoingState, incomingState, function.apply(incomingState));
            }
        }
    }

    public void addMappedTransition(List<Pair<S, S>> map) {
        addMappedTransition(map, none());
    }

    public void addMappedTransition(List<Pair<S, S>> map, Runnable runnable) {
        addMappedTransition(map, runOnce(runnable));
    }

    public void addMappedTransition(List<Pair<S, S>> map, Command command) {
        map.forEach((Pair<S, S> pair) -> addTransition(pair.getFirst(), pair.getSecond(), command));
    }

    public void addMappedTransition(List<Pair<S, S>> map, Function<S, Command> function) {
        map.forEach((Pair<S, S> pair) -> addTransition(pair.getFirst(), pair.getSecond(), function));
    }

    public void addUndefinedState(S undefinedState, S exitState)  {
        addUndefinedState(undefinedState, exitState, none());
    }

    public void addUndefinedState(S undefinedState, S exitState, Runnable runnable)  {
        addUndefinedState(undefinedState, exitState, runOnce(runnable));
    }

    public void addUndefinedState(S undefinedState, S exitState, Runnable toUndefinedRunnable, Runnable toExitRunnable)  {
        addUndefinedState(undefinedState, exitState, runOnce(toUndefinedRunnable), runOnce(toExitRunnable));
    }

    public void addUndefinedState(S undefinedState, S exitState, Command command)  {
        addUndefinedState(undefinedState, exitState, command, command);
    }

    public void addUndefinedState(S undefinedState, S exitState, Command toUndefinedCommand, Command toExitCommand) {
        addConvergingTransition(undefinedState, toUndefinedCommand);
        addTransition(undefinedState, exitState, toExitCommand);
    }

    public void addUndefinedState(S undefinedState, S exitState, Function<S, Command> function) {
        addConvergingTransition(undefinedState, function);
        addTransition(undefinedState, exitState, function);
    }



    @SuppressWarnings("unchecked")
    private Transition<S> getAsTransition(int x, int y) {
        return (Transition<S>) adjacencyMap[x][y];
    }

    public S fromOrdinal(int ordinal) {
        return states.get(ordinal);
    }

    public void setTransition(Transition<S> transition) {
        if(transition != null && isEmpty) isEmpty = false;
        if(transition.getOutgoingState().ordinal() == transition.getIncomingState().ordinal()) return;
        adjacencyMap[transition.getOutgoingState().ordinal()][transition.getIncomingState().ordinal()] = transition;
    }

    public void removeTransition(S start, S end) {
        transitionCount--;
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
        isEmpty = true;
    }

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
