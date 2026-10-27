package common.hardware.motorcontroller;

import java.util.HashSet;
import java.util.LinkedList;

import common.core.controllers.PIDFFConfig;
import common.core.misc.NAR_Robot;
import common.hardware.motorcontroller.sim.MotorSimModel;
import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Team 3128's motor class replacement for {@link MotorController}
 * @since 2023 Charged Up
 * @author Mason Lam
 */
public abstract class NAR_Motor implements AutoCloseable {

    /**
     * Store conversion factors for motor
     */
    public static class MotorConfig {
        public final double distanceFactor;
        public final double timeFactor;
        public final int supplyLimit;
        public final int statorLimit;
        public final double voltageCompensation;
        public final boolean inverted;
        public final Neutral mode;
        public final StatusFrames statusFrames;

        /**
         * Creates a new motor config
         * @param distanceFactor Factor to multiply distance units by (per rotations) ie. input circumference for distance
         * @param timeFactor Factor to multiply time units by (per minutes) ie. use 60 for seconds, 1 for minutes
         * @param statorLimit The currentLimit for the stator
         * @param supplyLimit The currentLimit for the motor controller
         * @param voltageCompensation The voltage compensation for the motor
         * @param inverted Whether or not the motor is inverted.
         * @param mode Neutral mode for the motor.
         * @param statusFrames Status frames for the motor.
         */
        public MotorConfig(double distanceFactor, double timeFactor, int statorLimit, int supplyLimit, double voltageCompensation, boolean inverted, Neutral mode, StatusFrames statusFrames) {
            this.distanceFactor = distanceFactor;
            this.timeFactor = timeFactor;
            this.statorLimit = statorLimit;
            this.supplyLimit = supplyLimit;
            this.voltageCompensation = voltageCompensation;
            this.inverted = inverted;
            this.mode = mode;
            this.statusFrames = statusFrames;
        }

        public MotorConfig(double distanceFactor, double timeFactor, int statorLimit, double voltageCompensation, boolean inverted, Neutral mode, StatusFrames statusFrames) {
            this(distanceFactor, timeFactor, statorLimit, 120, voltageCompensation, inverted, mode, statusFrames);
        }

        public MotorConfig(double distanceFactor, double timeFactor, int statorLimit, boolean inverted, Neutral mode, StatusFrames statusFrames) {
            this(distanceFactor, timeFactor, statorLimit, 120, 12, inverted, mode, statusFrames);
        }

        public MotorConfig(double distanceFactor, double timeFactor, int statorLimit, boolean inverted, Neutral mode) {
            this(distanceFactor, timeFactor, statorLimit, 120, 12, inverted, mode, StatusFrames.DEFAULT);
        }

        public MotorConfig invert() {
            return new MotorConfig(this.distanceFactor, this.timeFactor, this.statorLimit, this.supplyLimit, this.voltageCompensation, !this.inverted, this.mode, this.statusFrames);
        }

        public MotorConfig invertFollower() {
            return new MotorConfig(this.distanceFactor, this.timeFactor, this.statorLimit, this.supplyLimit, this.voltageCompensation, !this.inverted, this.mode, StatusFrames.FOLLOWER);
        }

        public MotorConfig follower() {
            return new MotorConfig(this.distanceFactor, this.timeFactor, this.statorLimit, this.supplyLimit, this.voltageCompensation, this.inverted, this.mode, StatusFrames.FOLLOWER);
        }

        @Override
        public String toString() {
            // Strings can't change, so concat returns a new one that has to be kept
            String output = "Motor Config";
            output = output.concat("\n Distance Factor: " + distanceFactor + ". \tEvery one true rotation of the motor will be measured as " + distanceFactor + " rotation(s).");
            output = output.concat("\n Time Factor: " + timeFactor + ". \tMotor velocity will be measured in units equivalent to " + (1 / timeFactor) + " rotations per minute.");
            output = output.concat("\n Stator Limit: " + statorLimit + ". \tThe maximum current applied to the stator is " + statorLimit + " amps. Max torque limited.");
            output = output.concat("\n Supply Limit: " + supplyLimit + ". \tThe maximum current applied to the motor is " + supplyLimit + " amps.");
            output = output.concat("\n Voltage Compensation: " + voltageCompensation + ". \tThe maximum voltage applied to the motor is " + voltageCompensation + " volts.");
            output = output.concat("\n Inversion: " + inverted + ". \t" + (inverted ? ("Clockwise is positive angular displacement.") : ("Counter-Clockwise is positive angular displacement.")));
            output = output.concat("\n Neutral Mode: " + mode.name() + ". \tWhen no output is applied the motor will " + mode.name().toLowerCase() + ".");
            output = output.concat("\n Status Frames: " + statusFrames.name() + ". \tThe motor will send status frames consistent with a " + statusFrames.name().toLowerCase() + " motor.");
            return output;
        }
    }

    public enum StatusFrames {
        DEFAULT,
        POSITION,
        VELOCITY,
        FOLLOWER;
    }

    /**
     * Motor control modes
     */
    public enum Control {
        PercentOutput,
        Velocity,
        Position;
    }


    /**
     * Motor states when no voltage is applied
     */
    public enum Neutral {
        BRAKE,
        COAST
    }

    private static final HashSet<NAR_Motor> leaders = new HashSet<NAR_Motor>();

    

    static {
        NAR_Robot.addPeriodic(()-> {
            for (final NAR_Motor leader : leaders) {
                final double output = leader.getAppliedOutput();
                for (final NAR_Motor follower : leader.followers) {
                    follower.set(output);
                }
            }
        }, 0.1);
    }

    private final LinkedList<NAR_Motor> followers = new LinkedList<NAR_Motor>();
    private final int id;
    private double prevValue = 0;
    private Control prevMode = Control.PercentOutput;
    private double prevFeedForward = 0;
    private double minInput;
    private double maxInput;
    private double relativePosition = 0;
    private boolean isContinuous = false;
    protected double unitConversionFactor = 1;
    protected double timeConversionFactor = 1;
    protected double motorRotationsPerSensorRotation = 1;
    

    public NAR_Motor(int id){
        this.id = id;
    }

    private static final LinkedList<NAR_Motor> simulatedMotors = new LinkedList<NAR_Motor>();
    protected MotorSimModel simModel;

    /**
     * Attaches physics so this motor moves in simulation. Does nothing on the real robot.
     * Only attach to the leader when several motors drive one mechanism, and give the
     * model a {@link DCMotor} for all of them, e.g. {@code DCMotor.getKrakenX60(2)}.
     * @param model The physics of what this motor drives
     */
    public void enableSim(MotorSimModel model) {
        if (!RobotBase.isSimulation()) return;
        simModel = model;
        simulatedMotors.add(this);
    }

    /**
     * Steps the physics of every motor with {@link #enableSim(MotorSimModel)}.
     * Call once per loop from {@code Robot.simulationPeriodic()}.
     * @param dtSeconds Time since the last call, normally 0.02
     */
    public static void updateSimulation(double dtSeconds) {
        for (final NAR_Motor motor : simulatedMotors) {
            motor.simulationStep(dtSeconds);
        }
    }

    /**
     * Reads this motor's simulated output voltage, steps {@link #simModel}, and writes the
     * new rotor position and velocity back to the motor controller's simulation.
     * @param dtSeconds Time step in seconds
     */
    protected void simulationStep(double dtSeconds) {}

    /**
     * Sets motor output power in volts
     * @param volts The voltage of the motor from -12 to 12
     */
    public void setVolts(double volts) {
        set(volts / 12.0);
    }

    /**
     * Sets motor output power
     * @param output The speed of the motor from -1 to 1
     */
    public void set(double output) {
        set(output, Control.PercentOutput);
    }

    /**
     * Sets the motor state
     * @param value Output of motor dependent on control mode
     * @param mode Type of control mode
     */
    public void set(double value, Control mode) {
        set(value, mode, 0);
    }

    /**
     * Sets the motor state with feedforward
     * @param value Output of motor dependent on control mode
     * @param mode Type of control mode
     * @param feedForward Feedforward of motor measured in volts
     */
    public void set(double value, Control mode, double feedForward) {
        if (value == prevValue && mode == prevMode && feedForward == prevFeedForward) return;
        prevValue = value;
        prevMode = mode;
        prevFeedForward = feedForward;
        switch(mode) {
            case PercentOutput:
                setPercentOutput(MathUtil.clamp(value, -1, 1));
                break;
            case Velocity:
                setVelocity(value / unitConversionFactor * timeConversionFactor, feedForward);
                break;
            case Position:
                if (isContinuous) {
                    final double position = getRawPosition() * unitConversionFactor;
                    final double errorBound = (maxInput - minInput) / 2.0;
                    final double error = convertInput(value) - convertInput(position);
                    final double delta = MathUtil.inputModulus(error, -errorBound, errorBound);
                    setPosition((position + delta) / unitConversionFactor, feedForward);
                    break;
                }
                setPosition(value / unitConversionFactor, feedForward);
                break;
        }
    }

    /**
     * Configures all major motor settings.
     * @param config Motor settings.
     */
    public void configMotor(MotorConfig config) {
        setUnitConversionFactor(config.distanceFactor);
        setTimeConversionFactor(config.timeFactor);
        setInvertedNoApply(config.inverted);
        setStatorLimitNoApply(config.statorLimit);
        setSupplyLimitNoApply(config.supplyLimit);
        enableVoltageCompensationNoApply(config.voltageCompensation);
        setNeutralModeNoApply(config.mode);
        switch(config.statusFrames) {
            case DEFAULT:
                setDefaultStatusFrames();
                break;
            case POSITION:
                setPositionStatusFrames();
                break;
            case VELOCITY:
                setVelocityStatusFrames();
                break;
            case FOLLOWER:
                setFollowerStatusFrames();
                break;
        }

        apply();

        Log.debug(Log.Type.MOTOR, "Motor (" + this.id + ")", config.toString());
    }

    public abstract void apply();

    /**
	 * Set the PID values for the controller.
	 * @param config PIDFFConfig containing kP, kI, and kD values.
	 */
	public abstract void configPID(PIDFFConfig config);

    public abstract void configPIDNoApply(PIDFFConfig config);

    /**
     * Loads PID and feedforward gains for the motor controller's own control loop, converting
     * them from this motor's units (set by {@link MotorConfig}) to the motor controller's units.
     * kG is applied as a constant, e.g. for an elevator.
     * @param gains kP, kI and kD in volts per unit of position or velocity error, with time in seconds;
     *              kS and kG in volts; kV in volts per unit of velocity
     * @param mode {@link Control#Position} or {@link Control#Velocity}
     */
    public void configClosedLoop(PIDFFConfig gains, Control mode) {
        configClosedLoop(gains, mode, 0);
    }

    /**
     * Like {@link #configClosedLoop(PIDFFConfig, Control)} for position control of an arm, where
     * kG is multiplied by the cosine of the arm's angle. Position 0 must be the arm held horizontal.
     * @param gains PID and feedforward gains, as in {@link #configClosedLoop(PIDFFConfig, Control)}
     * @param motorRotationsPerArmRotation Rotations of this motor's encoder per arm rotation: the
     *                                     gearing, or 1 for an absolute encoder on the arm's shaft
     */
    public void configArmClosedLoop(PIDFFConfig gains, double motorRotationsPerArmRotation) {
        configClosedLoop(gains, Control.Position, motorRotationsPerArmRotation);
    }

    private void configClosedLoop(PIDFFConfig gains, Control mode, double motorRotationsPerArmRotation) {
        // Comes first, since on a TalonFX it changes the motor controller's units
        setMechanismRatio(motorRotationsPerArmRotation > 0 ? motorRotationsPerArmRotation : 1);

        // This motor's units per one unit of the motor controller's own position or velocity
        final double unitsPerNativeVelocity = unitConversionFactor / timeConversionFactor * getNativeVelocityRPM();
        final double unitsPerNative = mode == Control.Velocity
            ? unitsPerNativeVelocity
            : unitConversionFactor * getNativePositionRotations();
        final double pidScale = unitsPerNative / getNativeOutputVolts();
        final double loopSeconds = getNativeTimeSeconds();

        // Feedforward is in volts on every motor controller, so only kV needs converting
        configNativeClosedLoop(mode,
            gains.kP * pidScale, gains.kI * pidScale * loopSeconds, gains.kD * pidScale / loopSeconds,
            gains.getkS(), gains.getkV() * unitsPerNativeVelocity, gains.getkG(), motorRotationsPerArmRotation);
    }

    /**
     * Loads gains already in the motor controller's own units.
     * @param motorRotationsPerArmRotation Gearing to an arm for a cosine kG, or 0 for a constant kG
     */
    protected abstract void configNativeClosedLoop(Control mode, double kP, double kI, double kD,
                                                   double kS, double kV, double kG, double motorRotationsPerArmRotation);

    /**
     * Makes the motor controller measure in mechanism rotations rather than motor rotations, for
     * motor controllers that need this for an arm's cosine kG. Does nothing by default.
     */
    protected void setMechanismRatio(double motorRotationsPerMechanismRotation) {}

    /** Rotor rotations in one unit of the motor controller's own position. */
    protected abstract double getNativePositionRotations();

    /** Rotor RPM in one unit of the motor controller's own velocity, e.g. 60 for rotations per second. */
    protected abstract double getNativeVelocityRPM();

    /** Volts in one unit of the motor controller's PID output, e.g. 1 for volts or 12 for output fraction. */
    protected abstract double getNativeOutputVolts();

    /** Seconds in the motor controller's time unit for kI and kD. */
    protected abstract double getNativeTimeSeconds();

    /**
     * Makes this motor copy {@code leader}'s output on the motor controller itself, so it
     * follows instantly instead of waiting for robot code.
     * @param leader The motor to follow
     * @param opposeLeader Whether this motor spins the opposite way to the leader
     * @return False if the two motor controllers can't follow each other, e.g. a Spark and a TalonFX
     */
    public boolean followOnController(NAR_Motor leader, boolean opposeLeader) {
        return false;
    }

    /**
     * Wraps a measurement value to the min and max input
     * @param value A measurement value
     * @return The measurement value wrapped by the min and max input
     */
    private double convertInput(double value) {
        return isContinuous ? MathUtil.inputModulus(value, minInput, maxInput) : value;
    }

    /** Enables continuous input.
    *
    * <p>Rather then using the max and min input range as constraints, the motor considers them to be the
    * same point and automatically calculates the shortest route to the setpoint.
    *
    * @param minInput The minimum value expected from the input.
    * @param maxInput The maximum value expected from the input.
    */
    public void enableContinuousInput(double minInput, double maxInput) {
        this.minInput = Math.min(minInput, maxInput);
        this.maxInput = Math.max(minInput, maxInput);
        isContinuous = true;
    }

    /**
     * Changes the units the motor measures position in, ie. rotations degrees
     * @param conversionFactor Conversion factor to change position units
     */
    public void setUnitConversionFactor(double conversionFactor) {
        this.unitConversionFactor = conversionFactor;
    }

    /**
     * Changes the units the motor measures time/velocity in, ie. rotations per minute rotations per second
     * @param conversionFactor Conversion factor to change time units
     */
    public void setTimeConversionFactor(double conversionFactor) {
        this.timeConversionFactor = conversionFactor;
    }

    /**
     * Resets the motor position. For an absolute encoder this moves its zero, e.g. after it slipped.
     * @param position Motor position, default units - rotations
     */
    public void resetPosition(double position) {
        resetRawPosition(position / unitConversionFactor);
    }

    /**
     * Tells the encoder where the mechanism is when the robot turns on. Unlike
     * {@link #resetPosition(double)}, this leaves an absolute encoder alone, since it already knows.
     * @param position Starting position, default units - rotations
     */
    public void setStartingPosition(double position) {
        resetPosition(position);
    }

    /**
     * Sets the motor inverted
     * @param inverted Inverts the motor
     */
    public abstract void setInverted(boolean inverted);

    public abstract void setInvertedNoApply(boolean inverted);

    /**
     * Sets motor output power
     * @param speed The speed of the motor from -1 to 1
     */
    protected abstract void setPercentOutput(double speed);

    /**
     * Sets the motor output based on its velocity
     * @param rpm Velocity of the motor in RPM
     * @param feedForward Feedforward component measured in volts
     */
    protected abstract void setVelocity(double rpm, double feedForward);

    /**
     * Sets motor output based on its position
     * @param rotations Position of the motor in rotations
     * @param feedForward Feedforward component measured in volts
     */
    protected abstract void setPosition(double rotations, double feedForward);

    public void resetRelativePosition() {
        this.relativePosition = getPosition();
    }

    /**
     * Resets the motor position
     * @param rotations Number of rotations
     */
    protected abstract void resetRawPosition(double rotations);

    /**
     * Gets the motors current output
     * @return Double measuring percent output from -1 to 1
     */
    public abstract double getAppliedOutput();

    /**
     * Returns the current going to the motor, increasing values means the motor is stalling
     * @return Double measuring the stall current of the motor
     */
    public abstract double getStallCurrent();

    /**
     * Returns the torque at the motor's shaft, before any gearing.
     * @return Torque in newton meters
     */
    public abstract double getTorque();

    /**
     * For an encoder that isn't on the motor, e.g. an absolute encoder on an arm's shaft: how many
     * times the motor turns per turn of the encoder. Simulation needs this to turn the motor's
     * motion into encoder readings. Call it before building the mechanism. 1 by default.
     * @param ratio Motor rotations per encoder rotation
     */
    public void setMotorRotationsPerSensorRotation(double ratio) {
        motorRotationsPerSensorRotation = ratio;
    }

    /** Motor rotations per rotation of the encoder this motor reads, 1 for the motor's own encoder. */
    public double getMotorRotationsPerSensorRotation() {
        return motorRotationsPerSensorRotation;
    }

    /**
     * Returns the current motor position, default unit - rotations
     * @return Double measuring motor position
     */
    public double getPosition() {
        final double position = getRawPosition() * unitConversionFactor;
        return convertInput(position);
    }

    public double getRelativePosition() {
        return getPosition() - this.relativePosition;
    }

    /**
     * Returns the current motor velocity, default unit - RPM
     * @return Double measuring motor velocity
     */
    public double getVelocity() {
        return getRawVelocity() * unitConversionFactor / timeConversionFactor;
    }

    /**
     * Returns the motor position in rotations
     * @return Double measuring motor position
     */
    protected abstract double getRawPosition();

    /**
     * Returns the current motor velocity in RPM
     * @return Double measuring motor velocity
     */
    protected abstract double getRawVelocity();

    /**
     * Returns the Motor's temperature in celsius.
     * @return The temperature in celsius.
     */
    public abstract double getTemperature();

    /**
     * Sets a motor's output based on the leader's
     * @param leader The motor to follow
     */
    public void follow(NAR_Motor leader) {
        leader.followers.add(this);
        leaders.add(leader);
        System.out.println(leader.followers.toString());
        System.out.println(leader.followers.size());
    }

    /**
     * Sets a motor's output based on the leader's
     * @param leader The motor to follow
     * @param config Motor settings
     */
    public void follow(NAR_Motor leader, MotorConfig config) {
        leader.followers.add(this);
        leaders.add(leader);
        configMotor(config.follower());
    }

    /**
     * Sets the motor's idle mode, its behavior when no voltage is applied
     * Followers will follow the leader's idle mode
     * @param mode Type of idle mode
     */
    public void setNeutralMode(Neutral mode) {
        switch(mode) {
            case BRAKE:
                setBrakeMode();
                break;
            case COAST:
                setCoastMode();
                break;
        }
        followers.forEach(follower -> follower.setNeutralMode(mode));
    }

    public void setNeutralModeNoApply(Neutral mode) {
        switch (mode) {
            case BRAKE:
                setBrakeModeNoApply();
                break;
            case COAST:
                setCoastModeNoApply();
                break;
        }
        followers.forEach(follower -> follower.setNeutralModeNoApply(mode));
    }

    /**
     * Sets the motor in brake mode
     */
    protected abstract void setBrakeMode();

    protected abstract void setBrakeModeNoApply();

    /**
     * Sets the motor in coast mode
     */
    protected abstract void setCoastMode();

    protected abstract void setCoastModeNoApply();

    /**
     * Sets voltage compensation, keeps output consistent when battery is above x volts
     * @param volts The max volts the motor goes too
     */
    public abstract void enableVoltageCompensation(double volts);

    public abstract void enableVoltageCompensationNoApply(double volts);

    /**
	 * Sets the stator limit in Amps.
	 * @param limit The current limit in Amps.
	 */
    public abstract void setStatorLimit(int limit);

    public abstract void setStatorLimitNoApply(int limit);

    /**
	 * Sets the supply limit in Amps.
	 * @param limit The current limit in Amps.
	 */
    public abstract void setSupplyLimit(int limit);

    public abstract void setSupplyLimitNoApply(int limit);

    /**
	 * Returns motor and motor controller functionality.
	 * @return State of the motor controller and motor.
	 */
    public abstract NarwhalDashboard.State getState();

    /**
     * Set the status frame rate to Team 3128's defaults
     */
    public abstract void setDefaultStatusFrames();

    /**
     * Set the status frame rate to Team 3128's for a position motor
     */
    public abstract void setPositionStatusFrames();

    /**
     * Set the status frame rate to Team 3128's for a velocity motor
     */
    public abstract void setVelocityStatusFrames();

    /**
     * Set the status frame rate to Team 3128's for a follower motor
     */
    public abstract void setFollowerStatusFrames();

    /**
     * Returns the motor object controlling the motion
     * @return The motor ie. SparkMax, TalonFX
     */
    public abstract Object getMotor();
}
