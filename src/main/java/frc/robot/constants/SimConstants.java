package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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

    interface Elevator {
        double MASS_KG = Units.lbsToKilograms(19.15);
        Translation3d ORIGIN = new Translation3d(-0.267, 0, 0.153);
        double STAGE_1_MAX_EXTENSION_METERS = Units.inchesToMeters(20.75); // from base stage bottom plate to bottom plate on stage 1
        double STAGE_2_MAX_EXTENSION_METERS = Units.inchesToMeters(20.5); // from stage 1 bottom plate to bottom plate on stage 2
        double STAGE_2_TO_INTAKE = Units.inchesToMeters(29.5); // from stage 2 bottom plate to intake pivot point
    }

    interface ElevatorArm {
        double RETRACTED_MOI_KG_M_SQUARED = 5.07674197;
        double EXTENDED_MOI_KG_M_SQUARED = 18.505581775;
        Rotation2d ZEROED_POSITION_TO_HORIZONTAL = Rotation2d.fromDegrees(-15.5);
        Rotation2d STARTING_ANGLE = Rotation2d.fromDegrees(49.284);
        double LENGTH_METERS = Units.inchesToMeters(31.876);
    }

    interface IntakeArm {
        double LENGTH_METERS = Units.inchesToMeters(11.865);
        Rotation2d ZEROED_POSITION_TO_HORIZONTAL = Rotation2d.fromDegrees(74.463);
        Rotation2d STARTING_ANGLE = Rotation2d.fromDegrees(0).plus(ZEROED_POSITION_TO_HORIZONTAL);
    }
}
