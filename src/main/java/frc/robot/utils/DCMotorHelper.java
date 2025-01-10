package frc.robot.utils;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DCMotorHelper {
    public static DCMotor getMinion(final int numMotors) {
        return new DCMotor(
                12,
                3.1,
                200.46,
                4,
                Units.rotationsPerMinuteToRadiansPerSecond(7000),
                numMotors
        );
    }
}
