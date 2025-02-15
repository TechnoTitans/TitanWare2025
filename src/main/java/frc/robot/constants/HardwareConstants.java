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
            int CANrangeId,
            double gearing,
            double lowerLimitRots,
            double upperLimitRots,
            double spoolDiameterMeters
    ) {}

    public static final ElevatorConstants ELEVATOR = new ElevatorConstants(
            RobotMap.CanivoreCANBus,
            14,
            15,
            16,
            4,
            0,
            (Units.inchesToMeters(41.25) / (0.0508 * Math.PI)),
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
            225,
            -0.244384765625,
            0,
            0.19775390625
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
            0,
            60,
            Units.degreesToRotations(-146),
            0
    );

    public record IntakeConstants(
            String CANBus,
            int algaeRollerMotorID,
            int coralRollerMotorID,
            int coralCANRangeId,
            double algaeGearing,
            double coralGearing
    ) {}

    public static final IntakeConstants INTAKE = new IntakeConstants(
            RobotMap.RioCANBus,
            21,
            22,
            23,
            4,
            4
    );
}
