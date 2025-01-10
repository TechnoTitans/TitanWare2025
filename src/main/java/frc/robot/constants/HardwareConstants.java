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
}
