package frc.robot.utils.teleop;

import edu.wpi.first.math.util.Units;

public class SwerveSpeed {
    public enum Speeds {
        FAST(Units.feetToMeters(15), 2 * Math.PI),
        NORMAL(Units.feetToMeters(12.5), 1.7 * Math.PI),
        SLOW(Units.feetToMeters(5), 1 * Math.PI);

        private final double translationSpeed;
        private final double rotationSpeed;

        Speeds(final double translationSpeed, final double rotationSpeed) {
            this.translationSpeed = translationSpeed;
            this.rotationSpeed = rotationSpeed;
        }

        public double getTranslationSpeed() {
            return translationSpeed;
        }

        public double getRotationSpeed() {
            return rotationSpeed;
        }
    }
    private static Speeds swerveSpeed = Speeds.NORMAL;

    private SwerveSpeed() {}

    public static Speeds getSwerveSpeed() {
        return swerveSpeed;
    }

    public static void setSwerveSpeed(final Speeds swerveSpeed) {
        SwerveSpeed.swerveSpeed = swerveSpeed;
    }
}
