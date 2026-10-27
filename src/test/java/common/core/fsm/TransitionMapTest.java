package common.core.fsm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

class TransitionMapTest {

    enum State { A, B, C }

    private static final List<State> ALL = List.of(State.values());

    private final TransitionMap<State> map = new TransitionMap<>(State.class);

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @Test
    void newMapIsEmpty() {
        assertTrue(map.isEmpty());
        assertEquals(0, map.getTransitionCount());
    }

    @Test
    void addedTransitionCanBeFound() {
        final Command command = Commands.none();
        map.addTransition(State.A, State.B, command);

        assertSame(command, map.getTransition(State.A, State.B).getCommand());
        assertNull(map.getTransition(State.B, State.A));
        assertFalse(map.isEmpty());
        assertEquals(1, map.getTransitionCount());
    }

    @Test
    void transitionToTheSameStateIsIgnored() {
        map.addTransition(State.A, State.A, Commands.none());
        assertTrue(map.isEmpty());
    }

    @Test
    void addingATransitionAgainReplacesIt() {
        final Command second = Commands.none();
        map.addTransition(State.A, State.B, Commands.none());
        map.addTransition(State.A, State.B, second);

        assertEquals(1, map.getTransitionCount());
        assertSame(second, map.getTransition(State.A, State.B).getCommand());
    }

    @Test
    void removingAMissingTransitionDoesNotChangeTheCount() {
        map.addTransition(State.A, State.B, Commands.none());
        map.removeTransition(State.B, State.C);
        assertEquals(1, map.getTransitionCount());
    }

    @Test
    void removeAllTransitionsEmptiesTheMap() {
        map.addCommutativeTransition(ALL);
        map.removeAllTransitions();
        assertTrue(map.isEmpty());
        assertEquals(0, map.getTransitionCount());
    }

    @Test
    void removeTransitionsFromAndToAState() {
        map.addCommutativeTransition(ALL);

        map.removeAllTransitionsFromState(State.A);
        assertTrue(map.getOutgoingTransitions(State.A).isEmpty());

        map.removeAllTransitionsToState(State.B);
        assertTrue(map.getIncomingTransitions(State.B).isEmpty());

        // B -> A, B -> C and C -> A are left
        assertEquals(3, map.getTransitionCount());
        assertNotNull(map.getTransition(State.C, State.A));
        assertNull(map.getTransition(State.C, State.B));
    }

    @Test
    void convergingTransitionComesFromEveryOtherState() {
        map.addConvergingTransition(State.C);

        assertEquals(2, map.getTransitionCount());
        assertNotNull(map.getTransition(State.A, State.C));
        assertNotNull(map.getTransition(State.B, State.C));
    }

    @Test
    void divergingTransitionGoesToEveryOtherState() {
        map.addDivergingTransition(State.A);

        assertEquals(2, map.getTransitionCount());
        assertNotNull(map.getTransition(State.A, State.B));
        assertNotNull(map.getTransition(State.A, State.C));
    }

    @Test
    void commutativeTransitionGoesBothWays() {
        map.addCommutativeTransition(State.A, State.B);

        assertNotNull(map.getTransition(State.A, State.B));
        assertNotNull(map.getTransition(State.B, State.A));
        assertEquals(2, map.getTransitionCount());
    }

    @Test
    void commutativeTransitionBetweenAllStatesConnectsEveryPair() {
        map.addCommutativeTransition(ALL);
        assertEquals(6, map.getTransitionCount());
    }

    @Test
    void correspondenceTransitionsPairStatesInOrder() {
        map.addCorrespondenceTransitions(List.of(State.A, State.B), List.of(State.B, State.C));

        assertNotNull(map.getTransition(State.A, State.B));
        assertNotNull(map.getTransition(State.B, State.C));
        assertEquals(2, map.getTransitionCount());
    }

    @Test
    void mappedTransitionsFollowThePairs() {
        map.addMappedTransition(List.of(Pair.of(State.C, State.A), Pair.of(State.B, State.A)));

        assertNotNull(map.getTransition(State.C, State.A));
        assertNotNull(map.getTransition(State.B, State.A));
        assertEquals(2, map.getTransitionCount());
    }

    @Test
    void undefinedStateCanBeReachedFromAnywhereButOnlyLeadsToItsExit() {
        map.addUndefinedState(State.C, State.A);

        assertNotNull(map.getTransition(State.A, State.C));
        assertNotNull(map.getTransition(State.B, State.C));
        assertNotNull(map.getTransition(State.C, State.A));
        assertNull(map.getTransition(State.C, State.B));
    }

    @Test
    void functionTransitionsAreBuiltForTheStateBeingEntered() {
        final List<State> builtFor = new ArrayList<>();
        map.addDivergingTransition(State.A, state -> {
            builtFor.add(state);
            return Commands.none();
        });

        assertEquals(List.of(State.A, State.B, State.C), builtFor);
        assertEquals(2, map.getTransitionCount()); // A -> A was built but ignored
    }

    @Test
    void outgoingAndIncomingTransitionsAreListed() {
        map.addDivergingTransition(State.A);
        map.addTransition(State.B, State.C, Commands.none());

        assertEquals(2, map.getOutgoingTransitions(State.A).size());
        assertEquals(2, map.getIncomingTransitions(State.C).size());
        assertEquals(1, map.getIncomingTransitions(State.B).size());
    }

    @Test
    void transitionKnowsItsStates() {
        final Transition<State> transition = new Transition<>(State.A, State.B);

        assertEquals(State.A, transition.getOutgoingState());
        assertEquals(State.B, transition.getIncomingState());
        assertTrue(transition.isTransition(State.A, State.B));
        assertFalse(transition.isTransition(State.B, State.A));
        assertTrue(transition.equals(new Transition<>(State.A, State.B, Commands.none())));
        assertEquals("A -> B", transition.toString());
    }
}
