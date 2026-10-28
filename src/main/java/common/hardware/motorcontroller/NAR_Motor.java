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
 * One motor and its motor controller, whatever the brand. {@link NAR_TalonFX} and
 * {@link NAR_CANSpark} fill in the details for CTRE and REV hardware, so the rest of the code can
 * treat every motor the same way.
 *
 * <h2>Units</h2>
 * A motor's encoder counts rotations of the motor. {@link MotorConfig} converts those into units
 * that make sense for the mechanism, like meters or degrees, so {@link #getPosition()},
 * {@link #getVelocity()} and {@link #set(double, Control)} all use your units.
 *
 * <h2>Settings</h2>
 * Settings like current limits come in two versions: {@code setStatorLimit} sends the change to
 * the motor controller right away, and {@code setStatorLimitNoApply} only stores it, so that
 * {@link #configMotor(MotorConfig)} can send all of them at once with {@link #apply()}.
 *
 * <h2>Repeated commands</h2>
 * {@link #set(double, Control, double)} skips a command identical to the last one, so calling it
 * every loop with the same setpoint doesn't flood the CAN bus.
 *
 * @since 2023 Charged Up
 * @author Mason Lam
 */
public abstract class NAR_Motor implements AutoCloseable {

    /**
     * The hardware settings for a motor: what units it measures in, how much current it may
     * draw, which way is forward, and whether it brakes when stopped.
     */
    public static class MotorConfig {
        /** Your position units per rotation of the encoder; see the constructor. */
        public final double distanceFactor;
        /** How many of your time units are in a minute; see the constructor. */
        public final double timeFactor;
        public final int supplyLimit;
        public final int statorLimit;
        public final double voltageCompensation;
        public final boolean inverted;
        public final Neutral mode;
        public final StatusFrames statusFrames;

        /**
         * Creates a motor config.
         *
         * @param distanceFactor Your position units per rotation of the encoder, which is usually
         *                       the motor. For an arm in degrees on a 50:1 gearbox, {@code 360.0 / 50};
         *                       for an elevator in meters, the meters it rises per motor rotation.
         * @param timeFactor How many of your time units are in a minute: 60 for speeds per second,
         *                   1 for per minute.
         * @param statorLimit Most amps through the motor itself. This limits torque, so it stops
         *                    mechanisms pushing too hard against hard stops and game pieces.
         * @param supplyLimit Most amps the motor controller draws from the battery. This protects
         *                    the battery and breakers. Sparks don't have a true supply limit, so on a
         *                    Spark this sets REV's secondary current limit instead, which briefly cuts
         *                    output when the motor's current goes over it; see
         *                    {@link NAR_CANSpark#setSupplyLimit(int)}.
         * @param voltageCompensation Volts treated as full power, so output is the same as the
         *                            battery drains. Usually 12.
         * @param inverted Whether positive output spins it clockwise instead of counterclockwise,
         *                 looking at the motor's shaft.
         * @param mode Whether it brakes or coasts when told to stop.
         * @param statusFrames How often it reports its position and speed over CAN.
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

        /** A config with a 120 A supply limit. */
        public MotorConfig(double distanceFactor, double timeFactor, int statorLimit, double voltageCompensation, boolean inverted, Neutral mode, StatusFrames statusFrames) {
            this(distanceFactor, timeFactor, statorLimit, 120, voltageCompensation, inverted, mode, statusFrames);
        }

        /** A config with a 120 A supply limit and 12 V compensation. */
        public MotorConfig(double distanceFactor, double timeFactor, int statorLimit, boolean inverted, Neutral mode, StatusFrames statusFrames) {
            this(distanceFactor, timeFactor, statorLimit, 120, 12, inverted, mode, statusFrames);
        }

        /** A config with a 120 A supply limit, 12 V compensation and the default status frames. */
        public MotorConfig(double distanceFactor, double timeFactor, int statorLimit, boolean inverted, Neutral mode) {
            this(distanceFactor, timeFactor, statorLimit, 120, 12, inverted, mode, StatusFrames.DEFAULT);
        }

        /** A copy that spins the other way. */
        public MotorConfig invert() {
            return new MotorConfig(this.distanceFactor, this.timeFactor, this.statorLimit, this.supplyLimit, this.voltageCompensation, !this.inverted, this.mode, this.statusFrames);
        }

        /** A copy for a follower that spins the other way. */
        public MotorConfig invertFollower() {
            return new MotorConfig(this.distanceFactor, this.timeFactor, this.statorLimit, this.supplyLimit, this.voltageCompensation, !this.inverted, this.mode, StatusFrames.FOLLOWER);
        }

        /** A copy for a follower, which reports less often since nothing reads it. */
        public MotorConfig follower() {
            return new MotorConfig(this.distanceFactor, this.timeFactor, this.statorLimit, this.supplyLimit, this.voltageCompensation, this.inverted, this.mode, StatusFrames.FOLLOWER);
        }

        /** Every setting, with what it means, for printing at startup. */
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

    /**
     * How often a motor controller reports over CAN. Reporting only what's used, and only as
     * often as needed, keeps the CAN bus from getting too busy.
     */
    public enum StatusFrames {
        /** Everything at moderate rates. */
        DEFAULT,
        /** Position often, for mechanisms that move to positions. */
        POSITION,
        /** Speed often, for flywheels and rollers. */
        VELOCITY,
        /** Little, since nothing reads a follower. */
        FOLLOWER;
    }

    /**
     * What {@link #set(double, Control)}'s value means.
     */
    public enum Control {
        /** A fraction of full power, from -1 to 1. */
        PercentOutput,
        /** A speed for the motor controller's control loop to hold, in your units. */
        Velocity,
        /** A position for the motor controller's control loop to go to, in your units. */
        Position;
    }


    /**
     * What a motor does when told to stop.
     */
    public enum Neutral {
        /** Shorts the motor's windings so it resists turning: stops quickly and holds better. */
        BRAKE,
        /** Lets it spin freely, e.g. for a flywheel that should slow down on its own. */
        COAST
    }

    /** Motors that other motors copy with {@link #follow(NAR_Motor)}. */
    private static final HashSet<NAR_Motor> leaders = new HashSet<NAR_Motor>();



    // Followers that can't follow on their motor controller copy their leader's output here,
    // every 0.1 seconds
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
    // The last command sent, so an identical one can be skipped
    private double prevValue = 0;
    private Control prevMode = Control.PercentOutput;
    private double prevFeedForward = 0;
    // The range that wraps around, e.g. 0 to 360 degrees, when continuous input is on
    private double minInput;
    private double maxInput;
    private double relativePosition = 0;
    private boolean isContinuous = false;
    /** Your position units per encoder rotation; {@link MotorConfig#distanceFactor}. */
    protected double unitConversionFactor = 1;
    /** Your time units per minute; {@link MotorConfig#timeFactor}. */
    protected double timeConversionFactor = 1;
    protected double motorRotationsPerSensorRotation = 1;


    /** @param id The motor controller's CAN ID */
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
     * Drives the motor at a voltage, as a fraction of 12 V.
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
     * Drives the motor at a power, or has its motor controller go to a position or hold a speed.
     * @param value A fraction of full power, a position or a speed, depending on {@code mode}
     * @param mode What {@code value} means
     */
    public void set(double value, Control mode) {
        set(value, mode, 0);
    }

    /**
     * Like {@link #set(double, Control)}, adding volts of feedforward the motor controller can't
     * work out itself. It sends the volts along with the setpoint every cycle of its loop.
     * @param value A fraction of full power, a position or a speed, depending on {@code mode}
     * @param mode What {@code value} means
     * @param feedForward Extra volts; ignored for {@link Control#PercentOutput}
     */
    public void set(double value, Control mode, double feedForward) {
        // Sending the same command again changes nothing, and takes up room on the CAN bus
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
                    // Go the short way around, e.g. from 350 to 10 degrees through 0, not back through 180
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
     * Mechanisms call this for each of their motors.
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

    /** Sends every stored setting to the motor controller, after the {@code NoApply} methods. */
    public abstract void apply();

    /**
	 * Sends kP, kI and kD to the motor controller exactly as given, in its own units. Mechanisms
	 * use {@link #configClosedLoop(PIDFFConfig, Control)} instead, which converts from your units.
	 */
	public abstract void configPID(PIDFFConfig config);

    /** Stores kP, kI and kD like {@link #configPID(PIDFFConfig)}, without sending them yet. */
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

    /**
     * Converts gains from your units to the motor controller's. Each motor controller has its own:
     * a TalonFX measures in rotations and seconds and outputs volts, while a Spark measures in
     * rotations and minutes and outputs a fraction of 12 V. kI and kD also depend on how often
     * the motor controller's loop runs.
     */
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

    /** Wraps a position into the continuous input range, if continuous input is on. */
    private double convertInput(double value) {
        return isContinuous ? MathUtil.inputModulus(value, minInput, maxInput) : value;
    }

    /**
     * Treats {@code minInput} and {@code maxInput} as the same point, like 0 and 360 degrees, for
     * something that can spin all the way around. Positions are then read within that range, and
     * position setpoints take the shortest way around.
    *
    * @param minInput One end of the range, e.g. 0
    * @param maxInput The other end, e.g. 360
    */
    public void enableContinuousInput(double minInput, double maxInput) {
        this.minInput = Math.min(minInput, maxInput);
        this.maxInput = Math.max(minInput, maxInput);
        isContinuous = true;
    }

    /** Sets your position units per encoder rotation; see {@link MotorConfig#distanceFactor}. */
    public void setUnitConversionFactor(double conversionFactor) {
        this.unitConversionFactor = conversionFactor;
    }

    /** Sets how many of your time units are in a minute; see {@link MotorConfig#timeFactor}. */
    public void setTimeConversionFactor(double conversionFactor) {
        this.timeConversionFactor = conversionFactor;
    }

    /**
     * Tells the encoder where it is now, e.g. 0 when the mechanism is against its starting hard stop.
     * @param position In your units
     */
    public void resetPosition(double position) {
        resetRawPosition(position / unitConversionFactor);
    }

    /**
     * Sets which way it spins for positive output.
     * @param inverted True for clockwise, looking at the shaft; false for counterclockwise
     */
    public abstract void setInverted(boolean inverted);

    /** Stores the inversion like {@link #setInverted(boolean)}, without sending it yet. */
    public abstract void setInvertedNoApply(boolean inverted);

    /**
     * Drives the motor at a fraction of full power.
     * @param speed From -1 to 1
     */
    protected abstract void setPercentOutput(double speed);

    /**
     * Has the motor controller hold a speed.
     * @param rpm Speed of the encoder in rotations per minute
     * @param feedForward Extra volts
     */
    protected abstract void setVelocity(double rpm, double feedForward);

    /**
     * Has the motor controller go to a position.
     * @param rotations Position of the encoder in rotations
     * @param feedForward Extra volts
     */
    protected abstract void setPosition(double rotations, double feedForward);

    /** Marks the current position as 0 for {@link #getRelativePosition()}, without changing the encoder. */
    public void resetRelativePosition() {
        this.relativePosition = getPosition();
    }

    /**
     * Tells the encoder where it is now.
     * @param rotations Position of the encoder in rotations
     */
    protected abstract void resetRawPosition(double rotations);

    /**
     * What the motor controller is putting out right now.
     * @return A fraction of full power, from -1 to 1
     */
    public abstract double getAppliedOutput();

    /**
     * Amps through the motor itself, its stator current. It jumps up when the motor is pushing
     * hard without moving, e.g. against a hard stop or holding a game piece, which makes it handy
     * for detecting those.
     * @return Amps
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
     * Where the encoder is, in your units, wrapped into the continuous input range if that's on.
     */
    public double getPosition() {
        final double position = getRawPosition() * unitConversionFactor;
        return convertInput(position);
    }

    /** Position since the last {@link #resetRelativePosition()}, in your units. */
    public double getRelativePosition() {
        return getPosition() - this.relativePosition;
    }

    /**
     * How fast the encoder is turning, in your units, e.g. meters per second.
     */
    public double getVelocity() {
        return getRawVelocity() * unitConversionFactor / timeConversionFactor;
    }

    /**
     * Where the encoder is, in rotations.
     */
    protected abstract double getRawPosition();

    /**
     * How fast the encoder is turning, in rotations per minute.
     */
    protected abstract double getRawVelocity();

    /**
     * The motor's temperature in degrees Celsius. Motors slow down to protect themselves when
     * they get too hot, usually around 100.
     */
    public abstract double getTemperature();

    /**
     * Makes this motor copy {@code leader}'s output, from robot code every 0.1 seconds. Prefer
     * {@link #followOnController(NAR_Motor, boolean)}, which is instant; mechanisms use it for you.
     * @param leader The motor to copy
     */
    public void follow(NAR_Motor leader) {
        leader.followers.add(this);
        leaders.add(leader);
        System.out.println(leader.followers.toString());
        System.out.println(leader.followers.size());
    }

    /**
     * Like {@link #follow(NAR_Motor)}, also applying {@code config} with follower status frames.
     * @param leader The motor to copy
     * @param config Settings for this motor
     */
    public void follow(NAR_Motor leader, MotorConfig config) {
        leader.followers.add(this);
        leaders.add(leader);
        configMotor(config.follower());
    }

    /**
     * Sets whether it brakes or coasts when told to stop. Motors following it with
     * {@link #follow(NAR_Motor)} get the same setting.
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

    /** Stores the neutral mode like {@link #setNeutralMode(Neutral)}, without sending it yet. */
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

    /** Makes it brake when stopped. */
    protected abstract void setBrakeMode();

    protected abstract void setBrakeModeNoApply();

    /** Makes it coast when stopped. */
    protected abstract void setCoastMode();

    protected abstract void setCoastModeNoApply();

    /**
     * Sets the volts treated as full power, so the same output gives the same push as the battery
     * drains, as long as the battery stays above it.
     * @param volts Usually 12
     */
    public abstract void enableVoltageCompensation(double volts);

    /** Stores the voltage compensation, without sending it yet. */
    public abstract void enableVoltageCompensationNoApply(double volts);

    /**
	 * Sets the most amps through the motor itself, which limits its torque. Handy for pushing
	 * gently against a hard stop or holding a game piece without crushing it.
	 * @param limit Amps
	 */
    public abstract void setStatorLimit(int limit);

    /** Stores the stator limit, without sending it yet. */
    public abstract void setStatorLimitNoApply(int limit);

    /**
	 * Sets the most amps the motor controller draws from the battery, to protect the battery and
	 * breakers and avoid brownouts. On a Spark this is REV's secondary current limit instead; see
	 * {@link NAR_CANSpark#setSupplyLimit(int)}.
	 * @param limit Amps
	 */
    public abstract void setSupplyLimit(int limit);

    /** Stores the supply limit, without sending it yet. */
    public abstract void setSupplyLimitNoApply(int limit);

    /**
	 * Whether the motor controller is connected and working, for the dashboard.
	 */
    public abstract NarwhalDashboard.State getState();

    /** Reports at the team's default rates. */
    public abstract void setDefaultStatusFrames();

    /** Reports position often, for a mechanism that moves to positions. */
    public abstract void setPositionStatusFrames();

    /** Reports speed often, for a flywheel or roller. */
    public abstract void setVelocityStatusFrames();

    /** Reports little, for a follower nothing reads. */
    public abstract void setFollowerStatusFrames();

    /**
     * The vendor's own motor object, e.g. a {@code TalonFX} or {@code SparkMax}, for anything this
     * class doesn't cover. Cast it to the right type.
     */
    public abstract Object getMotor();
}
