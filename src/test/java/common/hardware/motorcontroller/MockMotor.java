package common.hardware.motorcontroller;

import common.core.controllers.PIDFFConfig;

/**
 * A motor for unit tests. It records every command instead of talking to hardware, and its
 * encoder reads whatever the test sets, or the sim model when {@link #step(double)} is called.
 *
 * <p>{@link #talonLike(int)} and {@link #sparkLike(int)} use the same units for their own
 * control loops as a TalonFX and a Spark, so gain conversion can be checked against both.
 */
public class MockMotor extends NAR_Motor {

    // Units of the motor controller's own control loop, see NAR_Motor.configClosedLoop
    private final boolean talonLike;
    private final boolean canFollow;

    // Encoder, in rotor rotations and rotor RPM
    public double rawPosition = 0;
    public double rawVelocity = 0;

    // Last command, in the motor controller's units. NaN until the first command of that kind.
    public Control lastMode = null;
    public double lastPercentOutput = Double.NaN;
    public double lastPositionRotations = Double.NaN;
    public double lastVelocityRPM = Double.NaN;
    public double lastFeedforward = Double.NaN;
    public int commandCount = 0;

    // Closed loop gains, in the motor controller's units
    public double nativeKP, nativeKI, nativeKD, nativeKS, nativeKV, nativeKG;
    public double armRatio = 0;
    public double mechanismRatio = 1;
    public int closedLoopConfigCount = 0;

    // Configuration
    public boolean inverted = false;
    public Neutral neutralMode = null;
    public int statorLimit = 0;
    public int supplyLimit = 0;
    public double voltageCompensation = 0;
    public StatusFrames statusFrames = null;
    public NAR_Motor leader = null;
    public boolean opposesLeader = false;

    private MockMotor(int id, boolean talonLike, boolean canFollow) {
        super(id);
        this.talonLike = talonLike;
        this.canFollow = canFollow;
    }

    /** Volts, rotations per second and seconds, with Arm_Cosine measured in arm rotations. */
    public static MockMotor talonLike(int id) {
        return new MockMotor(id, true, true);
    }

    /** Output fraction, rotations and RPM, and a 1 ms loop. */
    public static MockMotor sparkLike(int id) {
        return new MockMotor(id, false, true);
    }

    /** A TalonFX-like motor that can't follow another motor, like a TalonFX next to a Spark. */
    public static MockMotor cannotFollow(int id) {
        return new MockMotor(id, true, false);
    }

    /** Steps the sim model with the last percent output, as the robot's simulationPeriodic would. */
    public void step(double dtSeconds) {
        simulationStep(dtSeconds);
    }

    /** The voltage of the last percent output command. */
    public double lastVolts() {
        return lastPercentOutput * 12;
    }

    @Override
    protected void simulationStep(double dtSeconds) {
        if (simModel == null) return;
        final double volts = lastMode == Control.PercentOutput ? lastVolts() : 0;
        simModel.update(volts, dtSeconds);
        rawPosition = simModel.getRotorRotations() / motorRotationsPerSensorRotation;
        rawVelocity = simModel.getRotorRPS() * 60 / motorRotationsPerSensorRotation;
    }

    @Override
    protected void configNativeClosedLoop(Control mode, double kP, double kI, double kD,
                                          double kS, double kV, double kG, double motorRotationsPerArmRotation) {
        nativeKP = kP;
        nativeKI = kI;
        nativeKD = kD;
        nativeKS = kS;
        nativeKV = kV;
        nativeKG = kG;
        armRatio = motorRotationsPerArmRotation;
        closedLoopConfigCount++;
    }

    @Override
    protected void setMechanismRatio(double motorRotationsPerMechanismRotation) {
        if (talonLike) mechanismRatio = motorRotationsPerMechanismRotation;
    }

    @Override
    protected double getNativePositionRotations() {
        return talonLike ? mechanismRatio : 1;
    }

    @Override
    protected double getNativeVelocityRPM() {
        return talonLike ? 60 * mechanismRatio : 1;
    }

    @Override
    protected double getNativeOutputVolts() {
        return talonLike ? 1 : 12;
    }

    @Override
    protected double getNativeTimeSeconds() {
        return talonLike ? 1 : 0.001;
    }

    @Override
    public boolean followOnController(NAR_Motor leader, boolean opposeLeader) {
        if (!canFollow) return false;
        this.leader = leader;
        this.opposesLeader = opposeLeader;
        return true;
    }

    @Override
    protected void setPercentOutput(double speed) {
        lastMode = Control.PercentOutput;
        lastPercentOutput = speed;
        commandCount++;
    }

    @Override
    protected void setVelocity(double rpm, double feedForward) {
        lastMode = Control.Velocity;
        lastVelocityRPM = rpm;
        lastFeedforward = feedForward;
        commandCount++;
    }

    @Override
    protected void setPosition(double rotations, double feedForward) {
        lastMode = Control.Position;
        lastPositionRotations = rotations;
        lastFeedforward = feedForward;
        commandCount++;
    }

    @Override
    protected void resetRawPosition(double rotations) {
        rawPosition = rotations;
    }

    @Override
    protected double getRawPosition() {
        return rawPosition;
    }

    @Override
    protected double getRawVelocity() {
        return rawVelocity;
    }

    @Override
    public double getAppliedOutput() {
        return lastMode == Control.PercentOutput ? lastPercentOutput : 0;
    }

    @Override
    public void apply() {}

    @Override
    public void configPID(PIDFFConfig config) {
        configPIDNoApply(config);
    }

    @Override
    public void configPIDNoApply(PIDFFConfig config) {
        nativeKP = config.kP;
        nativeKI = config.kI;
        nativeKD = config.kD;
    }

    @Override
    public void setInverted(boolean inverted) {
        setInvertedNoApply(inverted);
    }

    @Override
    public void setInvertedNoApply(boolean inverted) {
        this.inverted = inverted;
    }

    @Override
    protected void setBrakeMode() {
        setBrakeModeNoApply();
    }

    @Override
    protected void setBrakeModeNoApply() {
        neutralMode = Neutral.BRAKE;
    }

    @Override
    protected void setCoastMode() {
        setCoastModeNoApply();
    }

    @Override
    protected void setCoastModeNoApply() {
        neutralMode = Neutral.COAST;
    }

    @Override
    public void enableVoltageCompensation(double volts) {
        enableVoltageCompensationNoApply(volts);
    }

    @Override
    public void enableVoltageCompensationNoApply(double volts) {
        voltageCompensation = volts;
    }

    @Override
    public void setStatorLimit(int limit) {
        setStatorLimitNoApply(limit);
    }

    @Override
    public void setStatorLimitNoApply(int limit) {
        statorLimit = limit;
    }

    @Override
    public void setSupplyLimit(int limit) {
        setSupplyLimitNoApply(limit);
    }

    @Override
    public void setSupplyLimitNoApply(int limit) {
        supplyLimit = limit;
    }

    @Override
    public void setDefaultStatusFrames() {
        statusFrames = StatusFrames.DEFAULT;
    }

    @Override
    public void setPositionStatusFrames() {
        statusFrames = StatusFrames.POSITION;
    }

    @Override
    public void setVelocityStatusFrames() {
        statusFrames = StatusFrames.VELOCITY;
    }

    @Override
    public void setFollowerStatusFrames() {
        statusFrames = StatusFrames.FOLLOWER;
    }

    @Override
    public double getStallCurrent() {
        return 0;
    }

    @Override
    public double getTorque() {
        return 0;
    }

    @Override
    public double getTemperature() {
        return 25;
    }

    @Override
    public DeviceState getState() {
        return DeviceState.RUNNING;
    }

    @Override
    public Object getMotor() {
        return this;
    }

    @Override
    public void close() {}
}
