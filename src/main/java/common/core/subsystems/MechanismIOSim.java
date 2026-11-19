package common.core.subsystems;

/**
 * Fills the inputs box in simulation. The reads are identical to the real robot's,
 * because the fake encoder values live inside the motor object itself (its sim
 * state), so the same getters answer with simulated facts. If a physics engine
 * such as Maple-Sim is ever added, stepping it would happen here.
 */
public class MechanismIOSim extends MechanismIOReal {
}
