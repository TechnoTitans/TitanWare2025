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
            int masterMotorId,
            int followerMotorId,
            int CANCoderId,
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
            0.01,
            30,
            Units.inchesToMeters(1)
    );

    public record ArmConstants(
            String CANBus,
            int motorId,
            int CANCoderId,
            double gearing,
            double CANCoderOffset,
            double lowerLimitRots,
            double upperLimitRots
    ) {}

    public static final ArmConstants ARM = new ArmConstants(
            RobotMap.CanivoreCANBus,
            14,
            15,
            200,
            112.5,
            0.900634765625,
            0.01
    );

    public record IntakeConstants(
            String CANBus,
            int intakePivotMotorID,
            int algaeRollerMotorID,
            int coralRollerMotorID,
            int intakePivotCANCoderId,
            double intakePivotCANCoderOffset,
            int CANRangeId,
            double pivotGearing,
            double algaeGearing,
            double coralGearing,
            double pivotLowerLimitRots,
            double pivotUpperLimitRots
    ) {}

    public static final IntakeConstants INTAKE = new IntakeConstants(
            RobotMap.CanivoreCANBus,
            17,
            18,
            19,
            20,
            0.5,
            21,
            60,
            4,
            4,
            0.01,
            15

    );
}
