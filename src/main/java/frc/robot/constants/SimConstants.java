package frc.robot.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import frc.robot.subsystems.drive.constants.SwerveConstants;
import org.photonvision.estimation.TargetModel;

public interface SimConstants {
    // Assume 2mOhm resistance for voltage drop calculation
    double FALCON_MOTOR_RESISTANCE = 0.002;

    interface CTRE {
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

    interface ElevatorArm {
        Translation3d ORIGIN = new Translation3d(-0.267, 0, 0.153);

        double LENGTH_METERS = Units.inchesToMeters(31.876);

        double RETRACTED_MOI_KG_M_SQUARED = 5.07674197;
        double EXTENDED_MOI_KG_M_SQUARED = 18.505581775;

        Rotation2d ZEROED_POSITION_TO_HORIZONTAL = Rotation2d.fromDegrees(-18);
        Rotation2d STARTING_ANGLE = Rotation2d.fromDegrees(49.284);
    }

    interface Elevator {
        double MASS_KG = Units.lbsToKilograms(19.15);

        double ORIGIN_OFFSET = Units.inchesToMeters(1.125);
        // from base stage bottom plate to bottom plate on stage 1
        double STAGE_1_MAX_EXTENSION_METERS = Units.inchesToMeters(20.75);

        // stage 1 plate thickness
        double STAGE_2_OFFSET = Units.inchesToMeters(0.25);
        // from stage 1 bottom plate to bottom plate on stage 2
        double STAGE_2_MAX_EXTENSION_METERS = Units.inchesToMeters(20.5);

        // from stage 2 bottom plate to intake pivot point
        double STAGE_2_TO_INTAKE = Units.inchesToMeters(29.5);
    }

    interface IntakeArm {
        Transform3d ORIGIN_OFFSET = new Transform3d(
                Units.inchesToMeters(0.25),
                0,
                0,
                Rotation3d.kZero
        );

        double LENGTH_METERS = Units.inchesToMeters(11.865);
        Rotation2d ZEROED_POSITION_TO_HORIZONTAL = Rotation2d.fromDegrees(62.838);
        Rotation2d STARTING_ANGLE = Rotation2d.fromDegrees(0)
                .plus(ZEROED_POSITION_TO_HORIZONTAL);
    }

    interface Intake {
        double HEIGHT_METERS = Units.inchesToMeters(9.98);
        double WIDTH_METERS = Units.inchesToMeters(15);

        Transform3d CORAL_OFFSET = new Transform3d(
                Units.inchesToMeters(8),
                0,
                (Intake.HEIGHT_METERS / 2) - Units.inchesToMeters(0.75),
                Rotation3d.kZero
        );
    }

    interface GroundIntakeArm {
        double LENGTH_METERS = Units.inchesToMeters(12.5);
        double WIDTH_METERS = Units.inchesToMeters(18.45);

        double RETRACTED_MOI_KG_M_SQUARED = 0.0176;

        Translation3d ORIGIN = new Translation3d(.330, 0, .1844);
        Rotation2d ZEROED_POSITION_TO_HORIZONTAL = Rotation2d.fromDegrees(90);
        Rotation2d STARTING_ANGLE = Rotation2d.fromDegrees(0)
                .plus(ZEROED_POSITION_TO_HORIZONTAL);

        Transform3d CORAL_OFFSET = new Transform3d(
                WIDTH_METERS/2+Units.inchesToMeters(4.5),
                0,
                -0.152/2,
                Rotation3d.kZero
        );
    }

    interface Vision {
        TargetModel CORAL_TRACKING_MODEL =
                new TargetModel(Units.inchesToMeters(4), Units.inchesToMeters(4), Units.inchesToMeters(7));
    }
}
