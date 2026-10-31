# State machines

A state machine is a subsystem that is always in one named state and changes state only through
transitions that have been allowed.

## Why use one

A subsystem such as an intake does a few distinct things: sit still, pull game pieces in, and push
them out. Writing those as states (NEUTRAL, INTAKE, OUTTAKE) makes the robot's behavior easy to read
and easy to show on a dashboard. It also prevents invalid combinations, because a request for a
transition that has not been allowed is ignored.

A state machine can also control other state machines. The robot's Superstructure, for example, has
states like SHOOT that set the intake, hopper and shooter states together.

## Classes

| Class | Purpose |
|---|---|
| [`FSMSubsystemBase`](FSMSubsystemBase.java) | The subsystem. Extend it and list your states in an enum. |
| [`Transition`](Transition.java) | One allowed move between two states, and the command that performs it. |
| [`TransitionMap`](TransitionMap.java) | Every allowed move, with shortcuts for adding many at once. |

## Creating a state machine

1. List the states in an enum.
2. Extend `FSMSubsystemBase` and build the subsystem's mechanisms in the constructor.
3. In `registerTransitions()`, add a transition for every allowed move, with the command that
   performs it.

```java
public class Intake extends FSMSubsystemBase<IntakeStates> {
    private static final TransitionMap<IntakeStates> transitionMap = new TransitionMap<>(IntakeStates.class);

    public Intake() {
        super(IntakeStates.class, transitionMap, IntakeStates.NEUTRAL);
        roller = new RollerMechanism(...);
        addMechanisms(roller);
    }

    public void registerTransitions() {
        // Any state can go to any other, running the roller at that state's power
        transitionMap.addCommutativeTransition(List.of(IntakeStates.values()),
            state -> roller.runCommand(state.getPower()));
    }
}
```

`intake.setState(IntakeStates.INTAKE)` then runs the transition's command. Requesting a move that is
not in the map does nothing and logs a warning, which keeps a subsystem from doing something
unplanned, such as extending an arm through the robot.

The current state is published to NetworkTables under `StateMachines/<name>` whenever it changes, so
it can be followed in AdvantageScope or a log file.

## Adding transitions

`TransitionMap` has shortcuts for common patterns:

| Method | Adds |
|---|---|
| `addTransition` | One state to another, such as NEUTRAL to INTAKE. |
| `addCommutativeTransition` | Both directions between states. Given one list, every state in it to every other. |
| `addConvergingTransition` | Many states into one, such as anything to NEUTRAL. With no list, all states. |
| `addDivergingTransition` | One state out to many, such as NEUTRAL to anything. With no list, all states. |
| `addCorrespondenceTransitions` | Pairs from two lists in order: the first of each, then the second of each. |
| `addMappedTransition` | A list of (from, to) pairs. |
| `addUndefinedState` | A state, such as a manual override, that anything can enter but that exits to only one state. |

Each has overloads that differ only in what the transition does:

| Argument | Behavior |
|---|---|
| None | Only the state changes. |
| `Runnable` | Runs once, such as `() -> roller.run(0.5)`. |
| `Command` | Uses the same command for every transition. |
| `Function<S, Command>` | Builds the command for the state being entered, such as `state -> arm.setSetpointCommand(state.getAngle())`, so one line covers states that each need a different setpoint. |
