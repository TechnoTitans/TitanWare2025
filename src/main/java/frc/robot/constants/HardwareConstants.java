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
}
