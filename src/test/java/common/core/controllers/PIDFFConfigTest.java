package common.core.controllers;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class PIDFFConfigTest {

    // Only the feedforward gains matter here, so the PID gains are zero.
    private static final double KP = 0, KI = 0, KD = 0;

    // Feedforward gains, picked so each term contributes a different, easy-to-spot amount.
    private static final double KS = 0.5; // static gain: volts to overcome friction
    private static final double KV = 2.0; // velocity gain: volts per unit of velocity
    private static final double KA = 0.1; // acceleration gain (unused by positionFF and velocityFF)
    private static final double KG = 0.3; // gravity gain: volts to hold against gravity

    // pidOutput only matters for its sign, which decides the direction kS pushes.
    private static final double PID_OUTPUT_FORWARD = 1.0;
    private static final double PID_OUTPUT_REVERSE = -1.0;

    private static final boolean AT_SETPOINT = true;
    private static final boolean NOT_AT_SETPOINT = false;

    private static final double VELOCITY_SETPOINT = 10.0;

    // Assertion tolerance for floating-point comparison.
    private static final double TOLERANCE = 1e-9;

    private final PIDFFConfig config = new PIDFFConfig(KP, KI, KD, KS, KV, KA, KG);

    @Test
    void positionFFPushesKsInTheDirectionOfPidOutputAndAddsKg() {
        // By default the gravity function returns 1, so the full kG is applied.
        assertEquals(KS + KG, config.positionFF(PID_OUTPUT_FORWARD, NOT_AT_SETPOINT), TOLERANCE);
        assertEquals(-KS + KG, config.positionFF(PID_OUTPUT_REVERSE, NOT_AT_SETPOINT), TOLERANCE);
    }

    @Test
    void positionFFDropsKsAtSetpointToAvoidDithering() {
        assertEquals(KG, config.positionFF(PID_OUTPUT_FORWARD, AT_SETPOINT), TOLERANCE);
    }

    @Test
    void velocityFFAddsKvTimesSetpointAndScalesKgByGravityFunction() {
        final double gravityFunctionValue = 0.5; // e.g. cos(angle) for a pivot arm
        config.setkG_Function(() -> gravityFunctionValue);

        final double expected = KS + KV * VELOCITY_SETPOINT + KG * gravityFunctionValue;

        assertEquals(expected, config.velocityFF(VELOCITY_SETPOINT, PID_OUTPUT_FORWARD, NOT_AT_SETPOINT), TOLERANCE);
    }

    @Test
    void shortConstructorsLeaveTheOtherGainsAtZero() {
        final PIDFFConfig pOnly = new PIDFFConfig(3);
        assertEquals(3, pOnly.getkP(), TOLERANCE);
        assertEquals(0, pOnly.getkI(), TOLERANCE);
        assertEquals(0, pOnly.getkS(), TOLERANCE);
        assertEquals(0, pOnly.getkG(), TOLERANCE);

        final PIDFFConfig noKA = new PIDFFConfig(1, 2, 3, KS, KV, KG);
        assertEquals(0, noKA.getkA(), TOLERANCE);
        assertEquals(KG, noKA.getkG(), TOLERANCE);
    }

    @Test
    void supplierGainsReadTheirLatestValue() {
        final double[] tunedKS = {0.1}; // e.g. a Shuffleboard widget
        config.setkS(() -> tunedKS[0]);
        tunedKS[0] = 0.4;
        assertEquals(0.4, config.getkS(), TOLERANCE);
    }

    @Test
    void changeCopiesEveryGainAndTheGravityFunction() {
        final PIDFFConfig other = new PIDFFConfig(1, 2, 3, 4, 5, 6, 7);
        other.setkG_Function(() -> 0.5);

        config.change(other);

        assertEquals(1, config.getkP(), TOLERANCE);
        assertEquals(2, config.getkI(), TOLERANCE);
        assertEquals(3, config.getkD(), TOLERANCE);
        assertEquals(4, config.getkS(), TOLERANCE);
        assertEquals(5, config.getkV(), TOLERANCE);
        assertEquals(6, config.getkA(), TOLERANCE);
        assertEquals(7, config.getkG(), TOLERANCE);
        assertEquals(0.5, config.getkG_Function().getAsDouble(), TOLERANCE);
    }

    @Test
    void numberSettersReplaceEachGain() {
        final PIDFFConfig config = new PIDFFConfig();
        config.setkI(1);
        config.setkD(2);
        config.setkS(3);
        config.setkV(4);
        config.setkG(5);

        assertEquals(1, config.getkI(), TOLERANCE);
        assertEquals(2, config.getkD(), TOLERANCE);
        assertEquals(3, config.getkS(), TOLERANCE);
        assertEquals(4, config.getkV(), TOLERANCE);
        assertEquals(5, config.getkG(), TOLERANCE);
    }
}
