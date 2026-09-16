package common.core.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * CTRE-generated-style swerve drivetrain hardcoded for SDS Mk5n modules specifically - the gear
 * ratios, wheel radius, and coupling ratio below are Mk5n numbers, not generic swerve values.
 * We're expecting to run Mk5n for the next 3-4 years, so this being Mk5n-specific is intentional;
 * if/when we switch modules, these constants (and MODULE_FACTORY's wiring of them) need to be
 * revisited for whatever module replaces it.
 */
public class CTRESwerveDrivetrain
        extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {

// CAN IDs, the Pigeon ID/bus, and CANcoder offsets are specific to this year's robot wiring and
// calibration, not to the module hardware itself, so they're passed in by the robot project
// instead of being hardcoded here.
    public record ModuleHardware(int driveId, int steerId, int encoderId, Angle encoderOffset) {}

    // Mk5n drive gear ratio options (see class doc above)
    // public: this is the single source of truth for the robot's physical drivetrain layout -
    // referenced by frc.team3128.Constants for PathPlanner's fallback RobotConfig instead of
    // keeping a second, easily-stale copy of these numbers in the robot project
    public static final double DRIVE_GEAR_RATIO_R1 = 7.03;
    public static final double DRIVE_GEAR_RATIO_R2 = 6.03;
    public static final double DRIVE_GEAR_RATIO_R3 = 5.27;
    public static final double DRIVE_GEAR_RATIO = DRIVE_GEAR_RATIO_R2;

    public static final double STEER_GEAR_RATIO = 287.0 / 11.0;

    public static final Distance WHEEL_RADIUS =
        Units.Meters.of(0.0508);


// TODO: add SDS MK5 coupling gear ratio here, ctre defines this as drive rotations / azimuth rotations maybe ask henry
    private static final double COUPLING_GEAR_RATIO = 3.375;

    public static final Distance WHEEL_BASE =
        Units.Inches.of(20.75);

    public static final Distance TRACK_WIDTH =
        Units.Inches.of(20.75);


//replaces old constants, ctre swerve module factory handles these separately from NAR_motor configs
    private static final boolean STEER_MOTOR_INVERTED = true;
    private static final boolean ENCODER_INVERTED = false;

  
// modules are all mounted with the bevel gear on the same side (not mirrored L/R), so every
// drive motor needs the same invert - a positive command should always spin all four wheels
// the same way, unlike a mirrored/tank-style mount which needs opposite signs per side
    private static final boolean DRIVE_MOTOR_INVERTED = false;


//motor config

    private static final TalonFXConfiguration DRIVE_CONFIG =
        new TalonFXConfiguration();

    private static final TalonFXConfiguration STEER_CONFIG =
        new TalonFXConfiguration();

    private static final CANcoderConfiguration ENCODER_CONFIG =
        new CANcoderConfiguration();

//closed loop gains

    private static final com.ctre.phoenix6.configs.Slot0Configs DRIVE_GAINS =
        new com.ctre.phoenix6.configs.Slot0Configs()
            .withKP(0.005)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(0.065026)
            .withKV(2.5725)
            .withKA(0.56562);

    // kP=0.5 with Voltage closed-loop output only produced 0.5V per full rotation of error,
    // far too little to overcome the ~26:1 steer gearbox's static friction - modules never turned.
    // 100/0.5 matches the gain CTRE's own generated swerve template uses for this exact
    // Voltage + CANcoder feedback setup.
    private static final com.ctre.phoenix6.configs.Slot0Configs STEER_GAINS =
        new com.ctre.phoenix6.configs.Slot0Configs()
            .withKP(100.0)
            .withKI(0.0)
            .withKD(0.5);

//pin explicitly instead of relying on the library default, matching CTRE's generated swerve template
    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;


//module factory

    private static final SwerveModuleConstantsFactory<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration> MODULE_FACTORY =
        new SwerveModuleConstantsFactory<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration>()
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withCouplingGearRatio(COUPLING_GEAR_RATIO)
            .withWheelRadius(WHEEL_RADIUS)
            .withDriveMotorGains(DRIVE_GAINS)
            .withSteerMotorGains(STEER_GAINS)
            .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
            .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
            .withDriveMotorInitialConfigs(DRIVE_CONFIG)
            .withSteerMotorInitialConfigs(STEER_CONFIG);

//constructor
public CTRESwerveDrivetrain(
        int pigeonId,
        String canBus,
        ModuleHardware frontLeft,
        ModuleHardware frontRight,
        ModuleHardware backLeft,
        ModuleHardware backRight) {
    super(
        TalonFX::new,
        TalonFX::new,
        CANcoder::new,
        new SwerveDrivetrainConstants()
            .withCANBusName(canBus)
            .withPigeon2Id(pigeonId),
        MODULE_FACTORY.createModuleConstants(
            frontLeft.steerId(),
            frontLeft.driveId(),
            frontLeft.encoderId(),
            frontLeft.encoderOffset(),
            WHEEL_BASE.div(2),
            TRACK_WIDTH.div(2),
            DRIVE_MOTOR_INVERTED,
            STEER_MOTOR_INVERTED,
            ENCODER_INVERTED
        ),
        MODULE_FACTORY.createModuleConstants(
            frontRight.steerId(),
            frontRight.driveId(),
            frontRight.encoderId(),
            frontRight.encoderOffset(),
            WHEEL_BASE.div(2),
            TRACK_WIDTH.div(-2),
            DRIVE_MOTOR_INVERTED,
            STEER_MOTOR_INVERTED,
            ENCODER_INVERTED
        ),
        MODULE_FACTORY.createModuleConstants(
            backLeft.steerId(),
            backLeft.driveId(),
            backLeft.encoderId(),
            backLeft.encoderOffset(),
            WHEEL_BASE.div(-2),
            TRACK_WIDTH.div(2),
            DRIVE_MOTOR_INVERTED,
            STEER_MOTOR_INVERTED,
            ENCODER_INVERTED
        ),
        MODULE_FACTORY.createModuleConstants(
            backRight.steerId(),
            backRight.driveId(),
            backRight.encoderId(),
            backRight.encoderOffset(),
            WHEEL_BASE.div(-2),
            TRACK_WIDTH.div(-2),
            DRIVE_MOTOR_INVERTED,
            STEER_MOTOR_INVERTED,
            ENCODER_INVERTED
        )
    );
}
public double[] getRawCancoderAngles() {
    return new double[] {
        getModule(0)
            .getEncoder()
            .getAbsolutePosition()
            .getValue()
            .in(Units.Degrees),

        getModule(1)
            .getEncoder()
            .getAbsolutePosition()
            .getValue()
            .in(Units.Degrees),

        getModule(2)
            .getEncoder()
            .getAbsolutePosition()
            .getValue()
            .in(Units.Degrees),

        getModule(3)
            .getEncoder()
            .getAbsolutePosition()
            .getValue()
            .in(Units.Degrees)
    };
}
}