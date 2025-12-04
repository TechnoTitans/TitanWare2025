package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class HardwareConstants {
    public enum CANBus {
        RIO("rio"),
        CANIVORE("CANivore");

        public final String name;
        CANBus(final String name) {
            this.name = name;
        }

        public com.ctre.phoenix6.CANBus toPhoenix6CANBus() {
            return new com.ctre.phoenix6.CANBus(name);
        }
    }

    public record GyroConstants(
            CANBus CANBus,
            int gyroId
    ) {}

    public static final GyroConstants GYRO = new GyroConstants(
            CANBus.CANIVORE,
            13
    );

    public record ElevatorConstants(
            CANBus CANBus,
            int rightMotorId,
            int leftMotorId,
            double gearing,
            double lowerLimitRots,
            double upperLimitRots,
            double spoolDiameterMeters
    ) {}

    public static final ElevatorConstants ELEVATOR = new ElevatorConstants(
            CANBus.CANIVORE,
            14,
            15,
            4,
            0,
            6.242,
            Units.inchesToMeters(2)
    );

    public record ElevatorArmConstants(
            CANBus CANBus,
            int motorId,
            int CANCoderId,
            double gearing,
            double CANCoderOffset,
            double lowerLimitRots,
            double upperLimitRots
    ) {}

    public static final ElevatorArmConstants ELEVATOR_ARM = new ElevatorArmConstants(
            CANBus.RIO,
            17,
            18,
            324,
            0.197265625,
            0,
            0.19091796875
    );

    public record IntakeArmConstants(
            CANBus CANBus,
            int pivotMotorID,
            int pivotCANCoderId,
            double pivotCANCoderOffset,
            double pivotGearing,
            double pivotLowerLimitRots,
            double pivotUpperLimitRots
    ) {}

    public static final IntakeArmConstants INTAKE_ARM = new IntakeArmConstants(
            CANBus.RIO,
            19,
            20,
            0.62939453125,
            60,
            -0.35,
            0
    );

    public record IntakeConstants(
            CANBus CANBus,
            int rollerMotorID,
            int coralTOFID,
            double rollerGearing
    ) {}

    public static final IntakeConstants INTAKE = new IntakeConstants(
            CANBus.RIO,
            21,
            23,
            10.0
    );

    public record GroundIntakeArmConstants(
            CANBus CANBus,
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
            CANBus.RIO,
            24,
            25,
            0,
            75,
            Units.degreesToRotations(-130),
            0,
            Units.inchesToMeters(17.561254),
            Units.inchesToMeters(9.77)
    );

    public record GroundIntakeConstants(
            CANBus CANBus,
            int wheelMotorID,
            int coralCANRangeID,
            double wheelGearing
    ) {}

    public static final GroundIntakeConstants GROUND_INTAKE = new GroundIntakeConstants(
            CANBus.RIO,
            26,
            27,
            3
    );
}
