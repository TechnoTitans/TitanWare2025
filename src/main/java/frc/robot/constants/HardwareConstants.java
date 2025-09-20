package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class HardwareConstants {
    public record GyroConstants(
            String CANBus,
            int gyroId
    ) {}

    public static final GyroConstants GYRO = new GyroConstants(
            RobotMap.CanivoreCANBus,
            13
    );

    public record ElevatorConstants(
            String CANBus,
            int rightMotorId,
            int leftMotorId,
            double gearing,
            double lowerLimitRots,
            double upperLimitRots,
            double spoolDiameterMeters
    ) {}

    public static final ElevatorConstants ELEVATOR = new ElevatorConstants(
            RobotMap.CanivoreCANBus,
            14,
            15,
            4,
            0,
            6.242,
            Units.inchesToMeters(2)
    );

    public record ElevatorArmConstants(
            String CANBus,
            int motorId,
            int CANCoderId,
            double gearing,
            double CANCoderOffset,
            double lowerLimitRots,
            double upperLimitRots
    ) {}

    public static final ElevatorArmConstants ELEVATOR_ARM = new ElevatorArmConstants(
            RobotMap.RioCANBus,
            17,
            18,
            324,
            0.197265625,
            0,
            0.19091796875
    );

    public record IntakeArmConstants(
            String CANBus,
            int intakePivotMotorID,
            int intakePivotCANCoderId,
            double intakePivotCANCoderOffset,
            double pivotGearing,
            double pivotLowerLimitRots,
            double pivotUpperLimitRots
    ) {}

    public static final IntakeArmConstants INTAKE_ARM = new IntakeArmConstants(
            RobotMap.RioCANBus,
            19,
            20,
            0.62939453125,
            60,
            -0.35,
            0
    );

    public record IntakeConstants(
            String CANBus,
            int rollerRollerMotorID,
            int coralTOFID,
            double rollerGearing
    ) {}

    public static final IntakeConstants INTAKE = new IntakeConstants(
            RobotMap.RioCANBus,
            21,
            23,
            10.0
    );

    public record GroundIntakeConstants(
            String CANBus,
            int rollerMotorID,
            int coralTOFID,
            double rollerGearing
    ) {}

    public static final GroundIntakeConstants GROUND_INTAKE = new GroundIntakeConstants(
            RobotMap.RioCANBus,
            24,
            25,
            2
    );

    public record GroundIntakeArmConstants(
            String CANBus,
            int pivotMotorID,
            int groundIntakePivotEncoderID,
            double groundIntakePivotEncoderOffset,
            double pivotGearing,
            double pivotLowerLimitRots,
            double pivotUpperLimitRots
    ) {}

    public static final GroundIntakeArmConstants GROUND_INTAKE_ARM = new GroundIntakeArmConstants(
            RobotMap.RioCANBus,
            26,
            27,
            0,
            133,
            -0.35,
            .14
    ); //TODO: Limits need to change
}
