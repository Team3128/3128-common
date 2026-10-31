package common.hardware.motorcontroller;

import java.util.HashSet;
import java.util.LinkedList;

import common.core.controllers.PIDFFConfig;
import common.core.misc.NAR_Robot;
import common.hardware.motorcontroller.sim.MotorSimModel;
import common.utility.Log;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * One motor and its motor controller, independent of brand. {@link NAR_TalonFX} and
 * {@link NAR_CANSpark} implement it for CTRE and REV hardware.
 *
 * <p>Positions and velocities are in the units set by {@link MotorConfig}. See {@code README.md}
 * in this package for how units, settings and repeated commands work.
 *
 * @since 2023 Charged Up
 * @author Mason Lam
 */
public abstract class NAR_Motor implements AutoCloseable {

    /**
     * The hardware settings for a motor: its measurement units, current limits, direction, and whether
     * it brakes when stopped.
     */
    public static class MotorConfig {
        /** Configured position units per rotation of the encoder; see the constructor. */
        public final double distanceFactor;
        /** The number of configured time units per minute; see the constructor. */
        public final double timeFactor;
        public final int supplyLimit;
        public final int statorLimit;
        public final double voltageCompensation;
        public final boolean inverted;
        public final Neutral mode;
        public final StatusFrames statusFrames;

        /**
         * Creates a motor configuration.
         *
         * @param distanceFactor Configured position units per rotation of the encoder, which is usually
         *                       the motor. For an arm in degrees on a 50:1 gearbox, {@code 360.0 / 50};
         *                       for an elevator in meters, the meters of rise per motor rotation.
         * @param timeFactor The number of configured time units per minute: 60 for speeds per second,
         *                   1 for per minute.
         * @param statorLimit Maximum amps through the motor. This limits torque, which prevents
         *                    mechanisms from pushing too hard against hard stops and game pieces.
         * @param supplyLimit Maximum amps the motor controller draws from the battery. This protects
         *                    the battery and breakers. Sparks do not have a true supply limit, so on a
         *                    Spark this sets REV's secondary current limit instead, which briefly cuts
         *                    output when the motor's current exceeds it; see
         *                    {@link NAR_CANSpark#setSupplyLimit(int)}.
         * @param voltageCompensation Volts treated as full power, so that output remains the same as the
         *                            battery drains. Usually 12.
         * @param inverted Whether positive output spins the motor clockwise instead of counterclockwise,
         *                 viewed from the motor's shaft.
         * @param mode Whether the motor brakes or coasts when stopped.
         * @param statusFrames How often the motor reports its position and speed over CAN.
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

        /** Creates a config with a 120 A supply limit. */
        public MotorConfig(double distanceFactor, double timeFactor, int statorLimit, double voltageCompensation, boolean inverted, Neutral mode, StatusFrames statusFrames) {
            this(distanceFactor, timeFactor, statorLimit, 120, voltageCompensation, inverted, mode, statusFrames);
        }

        /** Creates a config with a 120 A supply limit and 12 V compensation. */
        public MotorConfig(double distanceFactor, double timeFactor, int statorLimit, boolean inverted, Neutral mode, StatusFrames statusFrames) {
            this(distanceFactor, timeFactor, statorLimit, 120, 12, inverted, mode, statusFrames);
        }

        /** Creates a config with a 120 A supply limit, 12 V compensation and the default status frames. */
        public MotorConfig(double distanceFactor, double timeFactor, int statorLimit, boolean inverted, Neutral mode) {
            this(distanceFactor, timeFactor, statorLimit, 120, 12, inverted, mode, StatusFrames.DEFAULT);
        }

        /** Returns a copy that spins in the opposite direction. */
        public MotorConfig invert() {
            return new MotorConfig(this.distanceFactor, this.timeFactor, this.statorLimit, this.supplyLimit, this.voltageCompensation, !this.inverted, this.mode, this.statusFrames);
        }

        /** Returns a copy for a follower that spins in the opposite direction. */
        public MotorConfig invertFollower() {
            return new MotorConfig(this.distanceFactor, this.timeFactor, this.statorLimit, this.supplyLimit, this.voltageCompensation, !this.inverted, this.mode, StatusFrames.FOLLOWER);
        }

        /** Returns a copy for a follower, which reports less often because nothing reads it. */
        public MotorConfig follower() {
            return new MotorConfig(this.distanceFactor, this.timeFactor, this.statorLimit, this.supplyLimit, this.voltageCompensation, this.inverted, this.mode, StatusFrames.FOLLOWER);
        }

        /** Returns every setting with its meaning, for printing at startup. */
        @Override
        public String toString() {
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

    /**
     * How often a motor controller reports over CAN. Reporting only what is used, and only as
     * often as necessary, reduces CAN bus load.
     */
    public enum StatusFrames {
        /** All signals at moderate rates. */
        DEFAULT,
        /** Frequent position reports, for mechanisms that move to positions. */
        POSITION,
        /** Frequent speed reports, for flywheels and rollers. */
        VELOCITY,
        /** Minimal reports, because nothing reads a follower. */
        FOLLOWER;
    }

    /**
     * The meaning of the value passed to {@link #set(double, Control)}.
     */
    public enum Control {
        /** A fraction of full power, from -1 to 1. */
        PercentOutput,
        /** A speed for the motor controller's control loop to hold, in the configured units. */
        Velocity,
        /** A position for the motor controller's control loop to reach, in the configured units. */
        Position;
    }


    /**
     * The behavior of a motor when stopped.
     */
    public enum Neutral {
        /** Shorts the motor's windings so that it resists turning, stopping quickly and holding better. */
        BRAKE,
        /** Allows the motor to spin freely, for example, for a flywheel that should slow down on its own. */
        COAST
    }

    /**
     * Motors that other motors copy with {@link #follow(NAR_Motor)}. A follower that cannot follow
     * on its motor controller copies its leader's output from here every 0.1 seconds.
     */
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
    /** The last value sent, so an identical command can be skipped. */
    private double prevValue = 0;
    /** The last mode sent. */
    private Control prevMode = Control.PercentOutput;
    /** The last feedforward sent. */
    private double prevFeedForward = 0;
    /** Lower end of the wrapping range, for example, 0 of 0 to 360 degrees, when continuous input is on. */
    private double minInput;
    /** Upper end of the wrapping range. */
    private double maxInput;
    private double relativePosition = 0;
    private boolean isContinuous = false;
    /** Configured position units per encoder rotation; {@link MotorConfig#distanceFactor}. */
    protected double unitConversionFactor = 1;
    /** Configured time units per minute; {@link MotorConfig#timeFactor}. */
    protected double timeConversionFactor = 1;
    protected double motorRotationsPerSensorRotation = 1;


    /** @param id The motor controller's CAN ID */
    public NAR_Motor(int id){
        this.id = id;
    }

    private static final LinkedList<NAR_Motor> simulatedMotors = new LinkedList<NAR_Motor>();
    protected MotorSimModel simModel;

    /**
     * Attaches a physics model so that this motor moves in simulation. Has no effect on the real robot.
     * When several motors drive one mechanism, attach the model only to the leader and give the
     * model a {@link DCMotor} for all of them, for example, {@code DCMotor.getKrakenX60(2)}.
     * @param model The physics of what this motor drives
     */
    public void enableSim(MotorSimModel model) {
        if (!RobotBase.isSimulation()) return;
        simModel = model;
        simulatedMotors.add(this);
    }

    /** Stops simulating this motor, for example, when it is closed. */
    protected void disableSim() {
        simulatedMotors.remove(this);
    }

    /**
     * Steps the physics of every motor with an attached {@link MotorSimModel}.
     * Call this once per loop from {@code Robot.simulationPeriodic()}.
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
     * Drives the motor at a voltage, expressed as a fraction of 12 V.
     * @param volts From -12 to 12
     */
    public void setVolts(double volts) {
        set(volts / 12.0);
    }

    /**
     * Drives the motor at a fraction of full power.
     * @param output From -1 (full reverse) to 1 (full forward)
     */
    public void set(double output) {
        set(output, Control.PercentOutput);
    }

    /**
     * Drives the motor at a power, or commands its motor controller to reach a position or hold a speed.
     * @param value A fraction of full power, a position or a speed, depending on {@code mode}
     * @param mode The meaning of {@code value}
     */
    public void set(double value, Control mode) {
        set(value, mode, 0);
    }

    /**
     * Equivalent to {@link #set(double, Control)}, adding volts of feedforward the motor controller cannot
     * calculate itself, which are sent with the setpoint on every cycle of its loop. A
     * command identical to the last one is skipped, to reduce CAN bus load.
     * @param value A fraction of full power, a position or a speed, depending on {@code mode}
     * @param mode The meaning of {@code value}
     * @param feedForward Additional volts; ignored for {@link Control#PercentOutput}
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
                    // Take the shortest path, for example, from 350 to 10 degrees through 0, not back through 180
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
     * Applies every setting in {@code config} and sends them to the motor controller together.
     * Mechanisms call this method for each of their motors.
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

    /** Sends every stored setting to the motor controller, after calls to the {@code NoApply} methods. */
    public abstract void apply();

    /**
	 * Sends kP, kI and kD to the motor controller exactly as given, in its own units. Mechanisms
	 * use {@link #configClosedLoop(PIDFFConfig, Control)} instead, which converts from the configured units.
	 */
	public abstract void configPID(PIDFFConfig config);

    /** Stores kP, kI and kD as {@link #configPID(PIDFFConfig)} does, without sending them. */
    public abstract void configPIDNoApply(PIDFFConfig config);

    /**
     * Loads PID and feedforward gains for the motor controller's own control loop, converting
     * them from this motor's units (set by {@link MotorConfig}) to the motor controller's units.
     * kG is applied as a constant, for example, for an elevator.
     * @param gains kP, kI and kD in volts per unit of position or velocity error, with time in seconds;
     *              kS and kG in volts; kV in volts per unit of velocity
     * @param mode {@link Control#Position} or {@link Control#Velocity}
     */
    public void configClosedLoop(PIDFFConfig gains, Control mode) {
        configClosedLoop(gains, mode, 0);
    }

    /**
     * Equivalent to {@link #configClosedLoop(PIDFFConfig, Control)} for position control of an arm, where
     * kG is multiplied by the cosine of the arm's angle. Position 0 must be the arm held horizontal.
     * @param gains PID and feedforward gains, as in {@link #configClosedLoop(PIDFFConfig, Control)}
     * @param motorRotationsPerArmRotation Rotations of this motor's encoder per arm rotation: the
     *                                     gearing, or 1 for an absolute encoder on the arm's shaft
     */
    public void configArmClosedLoop(PIDFFConfig gains, double motorRotationsPerArmRotation) {
        configClosedLoop(gains, Control.Position, motorRotationsPerArmRotation);
    }

    /**
     * Converts gains from the configured units to the motor controller's units. Each motor controller differs:
     * a TalonFX measures in rotations and seconds and outputs volts, while a Spark measures in
     * rotations and minutes and outputs a fraction of 12 V. kI and kD also depend on how often
     * the motor controller's loop runs.
     */
    private void configClosedLoop(PIDFFConfig gains, Control mode, double motorRotationsPerArmRotation) {
        // Must come first, because on a TalonFX it changes the motor controller's units
        setMechanismRatio(motorRotationsPerArmRotation > 0 ? motorRotationsPerArmRotation : 1);

        // This motor's units per one unit of the motor controller's own position or velocity
        final double unitsPerNativeVelocity = unitConversionFactor / timeConversionFactor * getNativeVelocityRPM();
        final double unitsPerNative = mode == Control.Velocity
            ? unitsPerNativeVelocity
            : unitConversionFactor * getNativePositionRotations();
        final double pidScale = unitsPerNative / getNativeOutputVolts();
        final double loopSeconds = getNativeTimeSeconds();

        // Feedforward is in volts on every motor controller, so only kV requires conversion
        configNativeClosedLoop(mode,
            gains.kP * pidScale, gains.kI * pidScale * loopSeconds, gains.kD * pidScale / loopSeconds,
            gains.getkS(), gains.getkV() * unitsPerNativeVelocity, gains.getkG(), motorRotationsPerArmRotation);
    }

    /**
     * Loads gains that are already in the motor controller's own units.
     * @param motorRotationsPerArmRotation Gearing to an arm for a cosine kG, or 0 for a constant kG
     */
    protected abstract void configNativeClosedLoop(Control mode, double kP, double kI, double kD,
                                                   double kS, double kV, double kG, double motorRotationsPerArmRotation);

    /**
     * Makes the motor controller measure in mechanism rotations rather than motor rotations, for
     * motor controllers that require this for an arm's cosine kG. Has no effect by default.
     */
    protected void setMechanismRatio(double motorRotationsPerMechanismRotation) {}

    /** Returns the rotor rotations in one unit of the motor controller's position. */
    protected abstract double getNativePositionRotations();

    /** Returns the rotor RPM in one unit of the motor controller's velocity, for example, 60 for rotations per second. */
    protected abstract double getNativeVelocityRPM();

    /** Returns the volts in one unit of the motor controller's PID output, for example, 1 for volts or 12 for output fraction. */
    protected abstract double getNativeOutputVolts();

    /** Returns the seconds in the motor controller's time unit for kI and kD. */
    protected abstract double getNativeTimeSeconds();

    /**
     * Makes this motor copy {@code leader}'s output on the motor controller itself, so that it
     * follows instantly rather than waiting for robot code.
     * @param leader The motor to follow
     * @param opposeLeader Whether this motor spins opposite to the leader
     * @return False if the two motor controllers cannot follow each other, for example, a Spark and a TalonFX
     */
    public boolean followOnController(NAR_Motor leader, boolean opposeLeader) {
        return false;
    }

    /** Wraps a position into the continuous input range, if continuous input is enabled. */
    private double convertInput(double value) {
        return isContinuous ? MathUtil.inputModulus(value, minInput, maxInput) : value;
    }

    /**
     * Treats {@code minInput} and {@code maxInput} as the same point, such as 0 and 360 degrees, for
     * a mechanism that can rotate continuously. Positions are then read within that range, and
     * position setpoints take the shortest path.
    *
    * @param minInput One end of the range, for example, 0
    * @param maxInput The other end, for example, 360
    */
    public void enableContinuousInput(double minInput, double maxInput) {
        this.minInput = Math.min(minInput, maxInput);
        this.maxInput = Math.max(minInput, maxInput);
        isContinuous = true;
    }

    /** Sets the configured position units per encoder rotation; see {@link MotorConfig#distanceFactor}. */
    public void setUnitConversionFactor(double conversionFactor) {
        this.unitConversionFactor = conversionFactor;
    }

    /** Sets the configured time units per minute; see {@link MotorConfig#timeFactor}. */
    public void setTimeConversionFactor(double conversionFactor) {
        this.timeConversionFactor = conversionFactor;
    }

    /**
     * Sets the encoder's current position, for example, 0 when the mechanism is against its starting hard stop.
     * @param position The position in the configured units
     */
    public void resetPosition(double position) {
        resetRawPosition(position / unitConversionFactor);
    }

    /**
     * Sets the direction of rotation for positive output.
     * @param inverted True for clockwise, viewed from the shaft; false for counterclockwise
     */
    public abstract void setInverted(boolean inverted);

    /** Stores the inversion as {@link #setInverted(boolean)} does, without sending it. */
    public abstract void setInvertedNoApply(boolean inverted);

    /**
     * Drives the motor at a fraction of full power.
     * @param speed From -1 to 1
     */
    protected abstract void setPercentOutput(double speed);

    /**
     * Commands the motor controller to hold a speed.
     * @param rpm Speed of the encoder in rotations per minute
     * @param feedForward Additional volts
     */
    protected abstract void setVelocity(double rpm, double feedForward);

    /**
     * Commands the motor controller to reach a position.
     * @param rotations Position of the encoder in rotations
     * @param feedForward Additional volts
     */
    protected abstract void setPosition(double rotations, double feedForward);

    /** Marks the current position as 0 for {@link #getRelativePosition()}, without changing the encoder. */
    public void resetRelativePosition() {
        this.relativePosition = getPosition();
    }

    /**
     * Sets the encoder's current position.
     * @param rotations Position of the encoder in rotations
     */
    protected abstract void resetRawPosition(double rotations);

    /**
     * Returns the current output of the motor controller.
     * @return A fraction of full power, from -1 to 1
     */
    public abstract double getAppliedOutput();

    /**
     * Returns the current through the motor, its stator current, in amps. It rises when the motor pushes
     * hard without moving, for example, against a hard stop or while holding a game piece, which makes it useful
     * for detecting those conditions.
     * @return Amps
     */
    public abstract double getStallCurrent();

    /**
     * Returns the torque at the motor's shaft, before any gearing.
     * @return Torque in newton meters
     */
    public abstract double getTorque();

    /**
     * Sets the gearing for an encoder that is not on the motor, for example, an absolute encoder on an arm's shaft: the
     * number of motor rotations per encoder rotation. Simulation uses this to convert the motor's
     * motion into encoder readings. Call this before building the mechanism. The default is 1.
     * @param ratio Motor rotations per encoder rotation
     */
    public void setMotorRotationsPerSensorRotation(double ratio) {
        motorRotationsPerSensorRotation = ratio;
    }

    /** Returns the motor rotations per rotation of this motor's encoder, 1 for the motor's own encoder. */
    public double getMotorRotationsPerSensorRotation() {
        return motorRotationsPerSensorRotation;
    }

    /**
     * Returns the encoder position in the configured units, wrapped into the continuous input range if enabled.
     */
    public double getPosition() {
        final double position = getRawPosition() * unitConversionFactor;
        return convertInput(position);
    }

    /** Returns the position since the last {@link #resetRelativePosition()}, in the configured units. */
    public double getRelativePosition() {
        return getPosition() - this.relativePosition;
    }

    /**
     * Returns the encoder speed in the configured units, for example, meters per second.
     */
    public double getVelocity() {
        return getRawVelocity() * unitConversionFactor / timeConversionFactor;
    }

    /**
     * Returns the encoder position in rotations.
     */
    protected abstract double getRawPosition();

    /**
     * Returns the encoder speed in rotations per minute.
     */
    protected abstract double getRawVelocity();

    /**
     * Returns the motor temperature in degrees Celsius. Motors reduce power to protect themselves when
     * they overheat, usually around 100 degrees.
     */
    public abstract double getTemperature();

    /**
     * Makes this motor copy {@code leader}'s output from robot code, every 0.1 seconds. Prefer
     * {@link #followOnController(NAR_Motor, boolean)}, which is instant; mechanisms use it automatically.
     * @param leader The motor to follow
     */
    public void follow(NAR_Motor leader) {
        leader.followers.add(this);
        leaders.add(leader);
        System.out.println(leader.followers.toString());
        System.out.println(leader.followers.size());
    }

    /**
     * Equivalent to {@link #follow(NAR_Motor)}, also applying {@code config} with follower status frames.
     * @param leader The motor to follow
     * @param config Settings for this motor
     */
    public void follow(NAR_Motor leader, MotorConfig config) {
        leader.followers.add(this);
        leaders.add(leader);
        configMotor(config.follower());
    }

    /**
     * Sets whether the motor brakes or coasts when stopped. Motors following it with
     * {@link #follow(NAR_Motor)} receive the same setting.
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

    /** Stores the neutral mode as {@link #setNeutralMode(Neutral)} does, without sending it. */
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

    /** Sets the motor to brake when stopped. */
    protected abstract void setBrakeMode();

    protected abstract void setBrakeModeNoApply();

    /** Sets the motor to coast when stopped. */
    protected abstract void setCoastMode();

    protected abstract void setCoastModeNoApply();

    /**
     * Sets the volts treated as full power, so that the same output gives the same result as the battery
     * drains, provided the battery stays above this voltage.
     * @param volts Usually 12
     */
    public abstract void enableVoltageCompensation(double volts);

    /** Stores the voltage compensation without sending it. */
    public abstract void enableVoltageCompensationNoApply(double volts);

    /**
	 * Sets the maximum amps through the motor, which limits its torque. This is useful for pushing
	 * gently against a hard stop or holding a game piece without crushing it.
	 * @param limit Amps
	 */
    public abstract void setStatorLimit(int limit);

    /** Stores the stator limit without sending it. */
    public abstract void setStatorLimitNoApply(int limit);

    /**
	 * Sets the maximum amps the motor controller draws from the battery, to protect the battery and
	 * breakers and avoid brownouts. On a Spark this is REV's secondary current limit instead; see
	 * {@link NAR_CANSpark#setSupplyLimit(int)}.
	 * @param limit Amps
	 */
    public abstract void setSupplyLimit(int limit);

    /** Stores the supply limit without sending it. */
    public abstract void setSupplyLimitNoApply(int limit);

    /**
	 * Returns whether the motor controller is connected and working, for the dashboard.
	 */
    public abstract DeviceState getState();

    /** Reports at the team's default rates. */
    public abstract void setDefaultStatusFrames();

    /** Reports position frequently, for a mechanism that moves to positions. */
    public abstract void setPositionStatusFrames();

    /** Reports speed frequently, for a flywheel or roller. */
    public abstract void setVelocityStatusFrames();

    /** Reports minimally, for a follower that nothing reads. */
    public abstract void setFollowerStatusFrames();

    /**
     * Returns the vendor's motor object, for example, a {@code TalonFX} or {@code SparkMax}, for functionality this
     * class does not cover. Cast it to the correct type.
     */
    public abstract Object getMotor();
}
