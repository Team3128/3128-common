package common.core.fsm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;
import java.util.function.Function;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Checks every shortcut in {@link TransitionMap} adds exactly the moves it says, whichever way
 * its transitions are given: nothing, a Runnable, a Command, or a Function.
 */
class TransitionMapShortcutsTest {

    enum State { A, B, C, D }

    private static final List<State> ALL = List.of(State.values());
    private static final List<State> AB = List.of(State.A, State.B);
    private static final List<State> CD = List.of(State.C, State.D);

    /** Each command is named after the state it was built for. */
    private static final Function<State, Command> NAMED = state -> Commands.none().withName(state.name());

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    /** Fills a new map with {@code add}, and lists its moves like "A -> B". */
    private static Set<String> moves(Consumer<TransitionMap<State>> add) {
        final TransitionMap<State> map = new TransitionMap<>(State.class);
        add.accept(map);
        final Set<String> moves = new TreeSet<>();
        for (State state : State.values()) {
            map.getOutgoingTransitions(state).forEach(transition -> moves.add(transition.toString()));
        }
        assertEquals(moves.size(), map.getTransitionCount());
        return moves;
    }

    private static Set<String> set(String... moves) {
        return new TreeSet<>(List.of(moves));
    }

    /** Every move from each of {@code from} to each of {@code to}, except to itself. */
    private static Set<String> everyMove(List<State> from, List<State> to) {
        final Set<String> moves = new TreeSet<>();
        for (State start : from) for (State end : to) if (start != end) moves.add(start + " -> " + end);
        return moves;
    }

    /** The name of the command run moving from {@code start} to {@code end}. */
    private static String commandFor(TransitionMap<State> map, State start, State end) {
        return map.getTransition(start, end).getCommand().getName();
    }

    @Test
    void singleTransitions() {
        final Set<String> ab = set("A -> B");
        assertEquals(ab, moves(map -> map.addTransition(State.A, State.B, () -> {})));
        assertEquals(ab, moves(map -> map.addTransition(State.A, State.B, Commands.none())));
        assertEquals(ab, moves(map -> map.addTransition(State.A, State.B, NAMED)));
        assertEquals(set("A -> B", "B -> C"), moves(map -> map.addTransitions(
            new Transition<>(State.A, State.B), new Transition<>(State.B, State.C))));
    }

    @Test
    void correspondenceTransitionsPairTheListsInOrder() {
        final Set<String> pairs = set("A -> C", "B -> D");
        assertEquals(pairs, moves(map -> map.addCorrespondenceTransitions(AB, CD)));
        assertEquals(pairs, moves(map -> map.addCorrespondenceTransitions(AB, CD, () -> {})));
        assertEquals(pairs, moves(map -> map.addCorrespondenceTransitions(AB, CD, Commands.none())));
        assertEquals(pairs, moves(map -> map.addCorrespondenceTransitions(AB, CD, NAMED)));
    }

    @Test
    void convergingTransitionsComeInto() {
        // From every state into one
        final Set<String> intoA = everyMove(ALL, List.of(State.A));
        assertEquals(intoA, moves(map -> map.addConvergingTransition(State.A)));
        assertEquals(intoA, moves(map -> map.addConvergingTransition(State.A, () -> {})));
        assertEquals(intoA, moves(map -> map.addConvergingTransition(State.A, NAMED)));

        // From some states into one
        final Set<String> abIntoC = everyMove(AB, List.of(State.C));
        assertEquals(abIntoC, moves(map -> map.addConvergingTransition(AB, State.C)));
        assertEquals(abIntoC, moves(map -> map.addConvergingTransition(AB, State.C, () -> {})));
        assertEquals(abIntoC, moves(map -> map.addConvergingTransition(AB, State.C, NAMED)));

        // From every state into each of several
        final Set<String> intoCD = everyMove(ALL, CD);
        assertEquals(intoCD, moves(map -> map.addConvergingTransition(CD)));
        assertEquals(intoCD, moves(map -> map.addConvergingTransition(CD, () -> {})));
        assertEquals(intoCD, moves(map -> map.addConvergingTransition(CD, NAMED)));

        // From some states into each of several
        final Set<String> abIntoCD = everyMove(AB, CD);
        assertEquals(abIntoCD, moves(map -> map.addConvergingTransition(AB, CD)));
        assertEquals(abIntoCD, moves(map -> map.addConvergingTransition(AB, CD, () -> {})));
        assertEquals(abIntoCD, moves(map -> map.addConvergingTransition(AB, CD, NAMED)));
    }

    @Test
    void convergingFunctionsBuildTheCommandForTheStateEntered() {
        final TransitionMap<State> map = new TransitionMap<>(State.class);
        map.addConvergingTransition(AB, CD, NAMED);
        assertEquals("C", commandFor(map, State.A, State.C));
        assertEquals("D", commandFor(map, State.B, State.D));
    }

    @Test
    void divergingTransitionsGoOutTo() {
        // From one state to every other
        final Set<String> fromA = everyMove(List.of(State.A), ALL);
        assertEquals(fromA, moves(map -> map.addDivergingTransition(State.A)));
        assertEquals(fromA, moves(map -> map.addDivergingTransition(State.A, () -> {})));
        assertEquals(fromA, moves(map -> map.addDivergingTransition(State.A, Commands.none())));
        assertEquals(fromA, moves(map -> map.addDivergingTransition(State.A, NAMED)));

        // From one state to some
        final Set<String> aToCD = everyMove(List.of(State.A), CD);
        assertEquals(aToCD, moves(map -> map.addDivergingTransition(State.A, CD)));
        assertEquals(aToCD, moves(map -> map.addDivergingTransition(State.A, CD, () -> {})));
        assertEquals(aToCD, moves(map -> map.addDivergingTransition(State.A, CD, NAMED)));

        // From several states to every other
        final Set<String> fromAB = everyMove(AB, ALL);
        assertEquals(fromAB, moves(map -> map.addDivergingTransition(AB)));
        assertEquals(fromAB, moves(map -> map.addDivergingTransition(AB, () -> {})));
        assertEquals(fromAB, moves(map -> map.addDivergingTransition(AB, Commands.none())));
        assertEquals(fromAB, moves(map -> map.addDivergingTransition(AB, NAMED)));

        // From several states to some
        final Set<String> abToCD = everyMove(AB, CD);
        assertEquals(abToCD, moves(map -> map.addDivergingTransition(AB, CD)));
        assertEquals(abToCD, moves(map -> map.addDivergingTransition(AB, CD, () -> {})));
        assertEquals(abToCD, moves(map -> map.addDivergingTransition(AB, CD, NAMED)));
    }

    @Test
    void commutativeTransitionsGoBothWays() {
        final Set<String> ab = set("A -> B", "B -> A");
        assertEquals(ab, moves(map -> map.addCommutativeTransition(State.A, State.B, () -> {})));
        assertEquals(ab, moves(map -> map.addCommutativeTransition(State.A, State.B, () -> {}, () -> {})));
        assertEquals(ab, moves(map -> map.addCommutativeTransition(State.A, State.B, NAMED)));

        final Set<String> abWithC = set("A -> C", "C -> A", "B -> C", "C -> B");
        assertEquals(abWithC, moves(map -> map.addCommutativeTransition(AB, State.C)));
        assertEquals(abWithC, moves(map -> map.addCommutativeTransition(AB, State.C, () -> {})));
        assertEquals(abWithC, moves(map -> map.addCommutativeTransition(AB, State.C, () -> {}, () -> {})));

        final Set<String> cWithAB = set("C -> A", "A -> C", "C -> B", "B -> C");
        assertEquals(cWithAB, moves(map -> map.addCommutativeTransition(State.C, AB)));
        assertEquals(cWithAB, moves(map -> map.addCommutativeTransition(State.C, AB, () -> {})));
        assertEquals(cWithAB, moves(map -> map.addCommutativeTransition(State.C, AB, () -> {}, () -> {})));

        final Set<String> everyPair = everyMove(ALL, ALL);
        assertEquals(everyPair, moves(map -> map.addCommutativeTransition(ALL, () -> {})));
        assertEquals(everyPair, moves(map -> map.addCommutativeTransition(ALL, Commands.none())));
    }

    @Test
    void commutativeTransitionsCanRunDifferentCommandsEachWay() {
        final TransitionMap<State> map = new TransitionMap<>(State.class);
        map.addCommutativeTransition(State.A, State.B, Commands.none().withName("out"), Commands.none().withName("back"));
        assertEquals("out", commandFor(map, State.A, State.B));
        assertEquals("back", commandFor(map, State.B, State.A));
    }

    @Test
    void mappedTransitionsFollowThePairs() {
        final List<Pair<State, State>> pairs = List.of(Pair.of(State.A, State.D), Pair.of(State.C, State.B));
        final Set<String> expected = set("A -> D", "C -> B");
        assertEquals(expected, moves(map -> map.addMappedTransition(pairs, () -> {})));
        assertEquals(expected, moves(map -> map.addMappedTransition(pairs, NAMED)));
    }

    @Test
    void undefinedStateCanBeEnteredFromAnywhereButLeftOneWay() {
        // D is entered from anywhere, and only left to A
        final Set<String> expected = everyMove(ALL, List.of(State.D));
        expected.add("D -> A");
        assertEquals(expected, moves(map -> map.addUndefinedState(State.D, State.A, () -> {})));
        assertEquals(expected, moves(map -> map.addUndefinedState(State.D, State.A, () -> {}, () -> {})));
        assertEquals(expected, moves(map -> map.addUndefinedState(State.D, State.A, Commands.none())));
        assertEquals(expected, moves(map -> map.addUndefinedState(State.D, State.A, NAMED)));
    }

    @Test
    void runnablesRunOnceWhenTheirTransitionDoes() {
        final AtomicInteger runs = new AtomicInteger();
        final TransitionMap<State> map = new TransitionMap<>(State.class);
        map.addTransition(State.A, State.B, runs::incrementAndGet);

        // Commands only run while the robot is enabled
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        map.getTransition(State.A, State.B).execute();
        CommandScheduler.getInstance().run();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();

        assertEquals(1, runs.get());
        assertTrue(map.getTransition(State.A, State.B).isFinished());
    }

    @Test
    void lookupsAndListing() {
        final TransitionMap<State> map = new TransitionMap<>(State.class);
        map.addTransition(State.A, State.B, Commands.none());

        assertEquals(State.C, map.fromOrdinal(2));
        assertTrue(map.toString().contains("A -> B"));
    }
}
