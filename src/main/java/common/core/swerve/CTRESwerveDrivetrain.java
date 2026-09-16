package common.core.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class CTRESwerveDrivetrain
        extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {

//drivetrain constants

    private static final int PIGEON_ID = 9;
    private static final String CAN_BUS = "drivetrain";

    // SDS Mk5n drive gear ratio options
    private static final double DRIVE_GEAR_RATIO_R1 = 7.03;
    private static final double DRIVE_GEAR_RATIO_R2 = 6.03;
    private static final double DRIVE_GEAR_RATIO_R3 = 5.27;
    private static final double DRIVE_GEAR_RATIO = DRIVE_GEAR_RATIO_R2;

    private static final double STEER_GEAR_RATIO = 287.0 / 11.0;

    private static final Distance WHEEL_RADIUS =
        Units.Meters.of(0.0508);


// TODO: add SDS MK5 coupling gear ratio here, ctre defines this as drive rotations / azimuth rotations maybe ask henry
    private static final double COUPLING_GEAR_RATIO = 3.375;

    private static final Distance WHEEL_BASE =
        Units.Inches.of(20.75);

    private static final Distance TRACK_WIDTH =
        Units.Inches.of(20.75);


// IDs

    // Front Left
    private static final int FL_DRIVE_ID = 1;
    private static final int FL_STEER_ID = 2;
    private static final int FL_ENCODER_ID = 10;

    // Front Right
    private static final int FR_DRIVE_ID = 3;
    private static final int FR_STEER_ID = 4;
    private static final int FR_ENCODER_ID = 11;

    // Back Left
    private static final int BL_DRIVE_ID = 5;
    private static final int BL_STEER_ID = 6;
    private static final int BL_ENCODER_ID = 12;

    // Back Right
    private static final int BR_DRIVE_ID = 7;
    private static final int BR_STEER_ID = 8;
    private static final int BR_ENCODER_ID = 13;


//put cancoder offsets here, just put them as zero for now

    private static final Angle FL_ENCODER_OFFSET =
        Units.Degrees.of(97.55859375);

    private static final Angle FR_ENCODER_OFFSET =
        Units.Degrees.of(7.91015625-180);

    private static final Angle BL_ENCODER_OFFSET =
        Units.Degrees.of(-18.896484375);

    private static final Angle BR_ENCODER_OFFSET =
        Units.Degrees.of(-47.63671875);




//replaces old constants, ctre swerve module factory handles these separately from NAR_motor configs
    private static final boolean STEER_MOTOR_INVERTED = true;
    private static final boolean ENCODER_INVERTED = false;

  
// replaces DRIVE_MOTOR_INVERTED = false for old constants, need to check if this should be true or false
    private static final boolean INVERT_LEFT_SIDE = false;


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

    private static final com.ctre.phoenix6.configs.Slot0Configs STEER_GAINS =
        new com.ctre.phoenix6.configs.Slot0Configs()
            .withKP(0.5)
            .withKI(0.0)
            .withKD(0.0);

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

//drivetrain constants

    private static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS =
        new SwerveDrivetrainConstants()
            .withCANBusName(CAN_BUS)
            .withPigeon2Id(PIGEON_ID);


//module constants

    private static final SwerveModuleConstants<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration> FRONT_LEFT =
        MODULE_FACTORY.createModuleConstants(
            FL_STEER_ID,
            FL_DRIVE_ID,
            FL_ENCODER_ID,
            FL_ENCODER_OFFSET,
            WHEEL_BASE.div(2),
            TRACK_WIDTH.div(2),
            INVERT_LEFT_SIDE,
            STEER_MOTOR_INVERTED,
            ENCODER_INVERTED
        );

    private static final SwerveModuleConstants<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration> FRONT_RIGHT =
        MODULE_FACTORY.createModuleConstants(
            FR_STEER_ID,
            FR_DRIVE_ID,
            FR_ENCODER_ID,
            FR_ENCODER_OFFSET,
            WHEEL_BASE.div(2),
            TRACK_WIDTH.div(-2),
            !INVERT_LEFT_SIDE,
            STEER_MOTOR_INVERTED,
            ENCODER_INVERTED
        );

    private static final SwerveModuleConstants<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration> BACK_LEFT =
        MODULE_FACTORY.createModuleConstants(
            BL_STEER_ID,
            BL_DRIVE_ID,
            BL_ENCODER_ID,
            BL_ENCODER_OFFSET,
            WHEEL_BASE.div(-2),
            TRACK_WIDTH.div(2),
            INVERT_LEFT_SIDE,
            STEER_MOTOR_INVERTED,
            ENCODER_INVERTED
        );

    private static final SwerveModuleConstants<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration> BACK_RIGHT =
        MODULE_FACTORY.createModuleConstants(
            BR_STEER_ID,
            BR_DRIVE_ID,
            BR_ENCODER_ID,
            BR_ENCODER_OFFSET,
            WHEEL_BASE.div(-2),
            TRACK_WIDTH.div(-2),
            !INVERT_LEFT_SIDE,
            STEER_MOTOR_INVERTED,
            ENCODER_INVERTED
        );


//constructor
public CTRESwerveDrivetrain() {
    super(
        TalonFX::new,
        TalonFX::new,
        CANcoder::new,
        DRIVETRAIN_CONSTANTS,
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
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