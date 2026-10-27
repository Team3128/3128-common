package common.hardware.motorcontroller;

import common.core.controllers.PIDFFConfig;
import common.utility.narwhaldashboard.NarwhalDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * A {@link NAR_Motor} backed by a WPILib {@link DCMotorSim} instead of hardware.
 * Mechanisms use it exactly like a {@link NAR_TalonFX}, so the real mechanism logic runs in simulation.
 *
 * <p>The physics is stepped lazily: every read or write first advances the model to the current
 * FPGA time, in fixed steps of {@value #STEP_SECONDS} seconds (any leftover time carries to the next call). Raw position and velocity are
 * reported at the motor (rotor) shaft, like the real controllers, so {@link MotorConfig} conversion
 * factors work unchanged.
 *
 * <p>Brake/coast, supply limits and temperature are not modeled.
 */
public class NAR_SimMotor extends NAR_Motor {

    private static final double STEP_SECONDS = 0.005;
    private static final double MAX_ELAPSED_SECONDS = 0.1;

    private final DCMotorSim sim;
    private final double gearing;
    private final PIDController pid = new PIDController(0, 0, 0, STEP_SECONDS);
    private PIDFFConfig ffConfig = new PIDFFConfig();

    private enum Mode { VOLTAGE, VELOCITY, POSITION }
    private Mode mode = Mode.VOLTAGE;
    private double target; // percent, rotor RPM or rotor rotations depending on mode
    private double feedForwardVolts;
    private double appliedVolts;
    private boolean inverted;
    private double statorLimit = Double.POSITIVE_INFINITY;
    private double voltageCompensation = 12;
    private double positionOffsetRotations;
    private double lastTime = Timer.getFPGATimestamp();
    private double pendingSeconds = 0;

    /**
     * @param id CAN id, for logging only
     * @param motor The motor model, e.g. {@code DCMotor.getKrakenX60(1)}
     * @param gearing Motor rotations per mechanism rotation
     * @param moiKgMetersSq Moment of inertia of everything the motor spins, measured at the mechanism shaft
     */
    public NAR_SimMotor(int id, DCMotor motor, double gearing, double moiKgMetersSq) {
        super(id);
        this.gearing = gearing;
        this.sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, moiKgMetersSq, gearing), motor);
    }

    /**
     * Moment of inertia of a solid cylinder (flywheel, roller) spinning about its axis.
     * @param massKg Mass in kilograms
     * @param radiusMeters Radius in meters
     * @return Inertia in kg*m^2
     */
    public static double solidCylinderMoi(double massKg, double radiusMeters) {
        return 0.5 * massKg * radiusMeters * radiusMeters;
    }

    /**
     * Equivalent rotational inertia, at the mechanism shaft, of a mass moving in a line.
     * @param massKg Mass in kilograms
     * @param metersPerRotation Linear distance the mass moves per mechanism rotation
     * @return Inertia in kg*m^2
     */
    public static double linearMoi(double massKg, double metersPerRotation) {
        final double metersPerRadian = metersPerRotation / (2 * Math.PI);
        return massKg * metersPerRadian * metersPerRadian;
    }

    /** Advances the model to now. */
    private void update() {
        final double now = Timer.getFPGATimestamp();
        pendingSeconds += MathUtil.clamp(now - lastTime, 0, MAX_ELAPSED_SECONDS);
        lastTime = now;
        while (pendingSeconds >= STEP_SECONDS) {
            appliedVolts = computeVolts();
            sim.setInputVoltage(appliedVolts);
            sim.update(STEP_SECONDS);
            pendingSeconds -= STEP_SECONDS;
        }
    }

    private double computeVolts() {
        double volts;
        switch (mode) {
            case VELOCITY:
                volts = pid.calculate(rotorRPM(), target) + feedForwardVolts;
                break;
            case POSITION:
                volts = pid.calculate(rotorRotations(), target) + feedForwardVolts;
                break;
            default:
                volts = target * voltageCompensation;
                break;
        }
        volts = MathUtil.clamp(volts, -voltageCompensation, voltageCompensation);
        // Stator limit: scale voltage down if the commanded current would exceed it.
        final double current = Math.abs(sim.getCurrentDrawAmps());
        if (current > statorLimit) volts *= statorLimit / current;
        return (inverted ? -1 : 1) * volts;
    }

    private double sign() {
        return inverted ? -1 : 1;
    }

    private double rotorRotations() {
        return sign() * sim.getAngularPositionRotations() * gearing + positionOffsetRotations;
    }

    private double rotorRPM() {
        return sign() * sim.getAngularVelocityRPM() * gearing;
    }

    @Override public void apply() {}

    @Override public void configPID(PIDFFConfig config) {
        configPIDNoApply(config);
    }

    @Override public void configPIDNoApply(PIDFFConfig config) {
        ffConfig = config;
        pid.setPID(config.kP, config.kI, config.kD);
    }

    @Override public void setInverted(boolean inverted) {
        update();
        this.inverted = inverted;
    }

    @Override public void setInvertedNoApply(boolean inverted) {
        setInverted(inverted);
    }

    @Override protected void setPercentOutput(double speed) {
        update();
        mode = Mode.VOLTAGE;
        target = speed;
    }

    @Override protected void setVelocity(double rpm, double feedForward) {
        update();
        if (mode != Mode.VELOCITY) pid.reset();
        mode = Mode.VELOCITY;
        target = rpm;
        feedForwardVolts = feedForward + ffConfig.getkS() * Math.signum(rpm) + ffConfig.getkV() * rpm / 60.0;
    }

    @Override protected void setPosition(double rotations, double feedForward) {
        update();
        if (mode != Mode.POSITION) pid.reset();
        mode = Mode.POSITION;
        target = rotations;
        feedForwardVolts = feedForward;
    }

    @Override protected void resetRawPosition(double rotations) {
        update();
        positionOffsetRotations = rotations - sign() * sim.getAngularPositionRotations() * gearing;
    }

    @Override public double getAppliedOutput() {
        update();
        return appliedVolts / 12.0 * sign();
    }

    @Override public double getStallCurrent() {
        update();
        return Math.abs(sim.getCurrentDrawAmps());
    }

    @Override public double getTorque() {
        update();
        return sim.getTorqueNewtonMeters();
    }

    @Override protected double getRawPosition() {
        update();
        return rotorRotations();
    }

    @Override protected double getRawVelocity() {
        update();
        return rotorRPM();
    }

    @Override public double getTemperature() {
        return 25;
    }

    @Override protected void setBrakeMode() {}
    @Override protected void setBrakeModeNoApply() {}
    @Override protected void setCoastMode() {}
    @Override protected void setCoastModeNoApply() {}

    @Override public void enableVoltageCompensation(double volts) {
        enableVoltageCompensationNoApply(volts);
    }

    @Override public void enableVoltageCompensationNoApply(double volts) {
        voltageCompensation = volts > 0 ? Math.min(volts, 12) : 12;
    }

    @Override public void setStatorLimit(int limit) {
        setStatorLimitNoApply(limit);
    }

    @Override public void setStatorLimitNoApply(int limit) {
        statorLimit = limit > 0 ? limit : Double.POSITIVE_INFINITY;
    }

    @Override public void setSupplyLimit(int limit) {}
    @Override public void setSupplyLimitNoApply(int limit) {}

    @Override public NarwhalDashboard.State getState() {
        return NarwhalDashboard.State.RUNNING;
    }

    @Override public void setDefaultStatusFrames() {}
    @Override public void setPositionStatusFrames() {}
    @Override public void setVelocityStatusFrames() {}
    @Override public void setFollowerStatusFrames() {}

    @Override public DCMotorSim getMotor() {
        return sim;
    }

    @Override public void close() {}
}
