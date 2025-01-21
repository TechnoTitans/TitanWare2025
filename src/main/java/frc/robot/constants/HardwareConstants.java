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
            Units.inchesToMeters(2)
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
            0.1,
            3
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
            RobotMap.CanivoreCANBus,
            17,
            20,
            0.5,
            60,
            0.01,
            15
    );

    public record IntakeConstants(
            String CANBus,
            int algaeRollerMotorID,
            int coralRollerMotorID,
            int coralCANRangeId,
            int algaeCANRangeId,
            double algaeGearing,
            double coralGearing
    ) {}

    public static final IntakeConstants INTAKE = new IntakeConstants(
            RobotMap.CanivoreCANBus,
            18,
            19,
            20,
            22,
            4,
            4
    );
}
