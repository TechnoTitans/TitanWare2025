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
}
