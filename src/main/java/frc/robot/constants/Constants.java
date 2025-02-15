package frc.robot.constants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public interface Constants {
    RobotMode CURRENT_MODE = RobotMode.REAL;
    CompetitionType CURRENT_COMPETITION_TYPE = CompetitionType.COMPETITION;
    double LOOP_PERIOD_SECONDS = 0.02;

    enum RobotMode {
        REAL,
        SIM,
        REPLAY,
        DISABLED
    }

    enum CompetitionType {
        TESTING,
        COMPETITION
    }

    interface NetworkTables {
        String AUTO_TABLE = "AutoSelector";
        String AUTO_PUBLISHER = "AutoOptions";
        String AUTO_SELECTED_SUBSCRIBER = "SelectedAuto";
    }

    interface Vision {
        Transform3d ROBOT_TO_FR_APRILTAG = new Transform3d(
                new Translation3d(Units.inchesToMeters(11.552), Units.inchesToMeters(-11.224), Units.inchesToMeters(8.617)),
                new Rotation3d(0, Units.degreesToRadians(-40), Units.rotationsToDegrees(30))
        );
        Transform3d ROBOT_TO_FL_TOP_APRILTAG = new Transform3d(
                new Translation3d(Units.inchesToMeters(2.331), Units.inchesToMeters(11.694), Units.inchesToMeters(13.717)),
                new Rotation3d(0, Units.degreesToRadians(-40), Units.degreesToRadians(0))
        );
        Transform3d ROBOT_TO_FL_BOTTOM_APRILTAG = new Transform3d(
                new Translation3d(Units.inchesToMeters(1.5), Units.inchesToMeters(11.759), Units.inchesToMeters(10.161)),
                new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0))
        );

        /**
         * Standard deviations of the supplied pose estimate (before vision, likely to be solely wheel odometry)
         */
        Vector<N3> STATE_STD_DEVS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(1));
        double VISION_CAMERA_DEFAULT_STD_DEV_FACTOR = 1.0;
        Vector<N3> VISION_STD_DEV_COEFFS = VecBuilder.fill(0.02, 0.02, 0.02);
        double MAX_ACCEPT_BEST_POSE_AMBIGUITY = 0.15;
    }
}
