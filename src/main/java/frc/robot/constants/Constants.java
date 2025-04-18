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
    CompetitionType CURRENT_COMPETITION_TYPE = CompetitionType.TESTING;
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

        String BRANCH_TABLE = "BranchSelector";
        String SELECTED_BRANCH = "BranchPublisher";
        String DASH_SELECTED_BRANCH = "SelectedBranch";
    }

    interface Vision {
        Transform3d ROBOT_TO_FR_APRILTAG = new Transform3d(
                new Translation3d(Units.inchesToMeters(11.78), Units.inchesToMeters(-10.622), Units.inchesToMeters(8.1085)),
                new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(10))
        );
        Transform3d ROBOT_TO_BL_APRILTAG = new Transform3d(
                new Translation3d(Units.inchesToMeters(-11.78), Units.inchesToMeters(10.622), Units.inchesToMeters(8.1085)),
                new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(-170))
        );
        Transform3d ROBOT_TO_FL_BOTTOM_APRILTAG = new Transform3d(
                new Translation3d(Units.inchesToMeters(0.792), Units.inchesToMeters(11.448), Units.inchesToMeters(10.11325)),
                new Rotation3d(0, 0, Units.degreesToRadians(-30))
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
