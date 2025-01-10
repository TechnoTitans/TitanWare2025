package frc.robot.constants;

public class HardwareConstants {
    public record GyroConstants(
            String CANBus,
            int gyroId
    ) {
    }

    public static final GyroConstants GYRO = new GyroConstants(
            RobotMap.CanivoreCANBus,
            13
    );

    public record IntakeConstants(
            String CANBus,
            int pivotMotorID,
            int algaeRollerMotorID,
            int coralRollerMotorID,
            int CANCoderId,
            int CANRangeId,
            double pivotGearing,
            double algaeGearing,
            double coralGearing,
            double lowerLimitRots,
            double upperLimitRots
    ) {}

    public static final IntakeConstants INTAKE = new IntakeConstants(
            RobotMap.CanivoreCANBus,
            17,
            18,
            19,
            20,
            21,
            60,
            4,
            4,
            0.01,
            15

    );
}
