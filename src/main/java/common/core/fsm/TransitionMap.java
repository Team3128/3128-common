package common.core.fsm;
import java.util.*;
import java.util.function.Function;

import common.utility.Log;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;

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

    public void addTransition(S start, S end, Runnable runnable) {}

    public void addTransition(S start, S end, Command command) {
        addTransition(new Transition<S>(start, end, command));
    }

    public void addTransition(S start, S end, Function<S, Command> function){}



    public void addCorrespondenceTransitions(List<S> start, List<S> end) {}
    
    public void addCorrespondenceTransitions(List<S> start, List<S> end, Runnable runnable) {}

    public void addCorrespondenceTransitions(List<S> start, List<S> end, Command command) {}

    public void addCorrespondenceTransitions(List<S> start, List<S> end, Function<S, Command> function) {
        if(start.size() != end.size()) Log.info("Transition Map", "Failed to load one to one transition");
        
        if(start.size() <= end.size()) {
            for(int i = 0; i < start.size(); i++) {
                addTransition(start.get(i), end.get(i), function.apply(start.get(i)));
            }
        }
        else {
            for(int i = 0; i < end.size(); i++) {
                addTransition(start.get(i), end.get(i), function.apply(start.get(i)));
            } 
        }
    }



    public void addConvergingTransition(S end) {}
    
    public void addConvergingTransition(S end, Runnable runnable) {
        addConvergingTransition(end, new InstantCommand(runnable));
    }
    
    public void addConvergingTransition(S end, Command command) {
        for (S s : states) {
            addTransition(s, end, command);
        }
    }

    public void addConvergingTransition(S end, Function<S, Command> function) {}

    public void addConvergingTransition(List<S> start, S end) {}

    public void addConvergingTransition(List<S> start, S end, Runnable runnable) {}

    public void addConvergingTransition(List<S> start, S end, Command command) {
        for (S outGoingState : start) {
            addTransition(outGoingState, end, command);
        }
    }

    public void addConvergingTransition(List<S> start, S end, Function<S, Command> function) {}

    public void addConvergingTransition(List<S> end) {}

    public void addConvergingTransition(List<S> end, Runnable runnable) {}

    public void addConvergingTransition(List<S> end, Command command) {}

    public void addConvergingTransition(List<S> end, Function<S, Command> function) {}

    public void addConvergingTransition(List<S> start, List<S> end) {}

    public void addConvergingTransition(List<S> start, List<S> end, Runnable runnable) {}

    public void addConvergingTransition(List<S> start, List<S> end, Command command) {}

    public void addConvergingTransition(List<S> start, List<S> end, Function<S, Command> function) {}



    public void addDivergingTransition(S start) {}

    public void addDivergingTransition(S start, Runnable runnable) {}

    public void addDivergingTransition(S start, Command command) {
        for (S s : states) {
            addTransition(start, s, command);
        }
    }

    public void addDivergingTransition(S start, Function<S, Command> function) {}

    public void addDivergingTransition(S start, List<S> end) {}

    public void addDivergingTransition(S start, List<S> end, Runnable runnable) {}

    public void addDivergingTransition(S start, List<S> end, Command command) {}
    
    public void addDivergingTransitions(S start, List<S> end, Function<S, Command> function) {}

    public void addDivergingTransition(List<S> start) {}

    public void addDivergingTransition(List<S> start, Runnable runnable) {}

    public void addDivergingTransition(List<S> start, Command command) {}

    public void addDivergingTransition(List<S> start, Function<S, Command> function) {}

    public void addDivergingTransition(List<S> start, List<S> end) {}

    public void addDivergingTransition(List<S> start, List<S> end, Runnable runnable) {}

    public void addDivergingTransition(List<S> start, List<S> end, Command command) {}

    public void addDivergingTransition(List<S> start, List<S> end, Function<S, Command> function) {}



    public void addCommutativeTransition(S start, S end) {}

    public void addCommutativeTransition(S start, S end, Runnable runnable) {}

    public void addCommutativeTransition(S start, S end, Runnable runnableForwards, Runnable runnableBackwards) {}

    public void addCommutativeTransition(S start, S end, Command command) {}

    public void addCommutativeTransition(S start, S end, Command commandForwards, Command commandBackwards) {}

    public void addCommutativeTransition(S start, S end, Function<S, Command> function) {}

    public void addCommutativeTransition(List<S> start, S end) {}

    public void addCommutativeTransition(List<S> start, S end, Runnable runnable) {}

    public void addCommutativeTransition(List<S> start, S end, Runnable runnableForwards, Runnable runnableBackwards) {}

    public void addCommutativeTransition(List<S> start, S end, Command command) {}

    public void addCommutativeTransition(List<S> start, S end, Command commandForwards, Command commandBackwards) {}

    public void addCommutativeTransition(List<S> start, S end, Function<S, Command> function) {}

    public void addCommutativeTransition(S start, List<S> end) {}

    public void addCommutativeTransition(S start, List<S> end, Runnable runnable) {}

    public void addCommutativeTransition(S start, List<S> end, Runnable runnableForwards, Runnable runnableBackwards) {}

    public void addCommutativeTransition(S start, List<S> end, Command command) {}

    public void addCommutativeTransition(S start, List<S> end, Command commandForwards, Command commandBackwards) {}

    public void addCommutativeTransition(S start, List<S> end, Function<S, Command> function) {}

    public void addCommutativeTransition(List<S> states) {}

    public void addCommutativeTransition(List<S> states, Runnable runnable) {}

    public void addCommutativeTransition(List<S> states, Command command) {}

    public void addCommutativeTransition(List<S> states, Function<S, Command> function) {
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
