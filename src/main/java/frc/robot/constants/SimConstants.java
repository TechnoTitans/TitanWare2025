package frc.robot.constants;

import frc.robot.subsystems.drive.constants.SwerveConstants;

public interface SimConstants {
    // Assume 2mOhm resistance for voltage drop calculation
    double FALCON_MOTOR_RESISTANCE = 0.002;

    interface CTRE {
        // TODO: verify that config calls in sim simply just take longer,
        //  and thus need a longer timeout than 0.05s (50ms)
        boolean DISABLE_NEUTRAL_MODE_IN_SIM = false;
        double CONFIG_TIMEOUT_SECONDS = 0.2;
    }

    interface SwerveModules {
        double WHEEL_RADIUS_M = SwerveConstants.Config.wheelRadiusMeters();
        double WHEEL_MASS_KG = 0.2313321; //0.51 lbs
        double DRIVE_WHEEL_MOMENT_OF_INERTIA_KG_M_SQUARED = WHEEL_MASS_KG * WHEEL_RADIUS_M * WHEEL_RADIUS_M;
        double TURN_WHEEL_MOMENT_OF_INERTIA_KG_M_SQUARED = 0.004;

        /**
         * Simulated drive voltage required to overcome friction.
         */
        double DRIVE_KS_VOLTS = 0.25;
        /**
         * Simulated steer voltage required to overcome friction.
         */
        double STEER_KS_VOLTS = 0.25;
    }
}
