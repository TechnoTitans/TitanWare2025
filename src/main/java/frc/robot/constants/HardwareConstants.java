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
            int pivotMotorID,
            int pivotCANCoderId,
            double pivotCANCoderOffset,
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

    public record GroundIntakeArmConstants(
            String CANBus,
            int pivotMotorID,
            int pivotCANCoderId,
            double pivotCANCoderOffset,
            double pivotGearing,
            double pivotLowerLimitRots,
            double pivotUpperLimitRots,
            double lengthMeters,
            double heightMeters
    ) {}

    // TODO change
    public static final GroundIntakeArmConstants GROUND_INTAKE_ARM = new GroundIntakeArmConstants(
            RobotMap.RioCANBus,
            24,
            25,
            0,
            75,
            Units.degreesToRotations(-130),
            0,
            Units.inchesToMeters(17.561254),
            Units.inchesToMeters(9.77)
    );
}
