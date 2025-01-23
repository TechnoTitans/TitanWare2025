package frc.robot.subsystems.superstructure;

public class Profiles {
    public static final SplineProfile L1_TO_L2 = new SplineProfile(
            0.5,
            Superstructure.Goal.L1,
            0.16,
            0.028,
            0.387,
            -0.047,
            Superstructure.Goal.L2
    );
    public static final SplineProfile L1_TO_L3 = new SplineProfile(
            0.7,
            Superstructure.Goal.L1,
            0.16,
            0.028,
            0.591,
            0,
            Superstructure.Goal.L3
    );
    public static final SplineProfile L1_TO_L4 = new SplineProfile(
            1.1,
            Superstructure.Goal.L1,
            0.579,
            0.075,
            0.922,
            0.261,
            Superstructure.Goal.L4
    );
    public static final SplineProfile L2_TO_L1 = new SplineProfile(
            0.57,
            Superstructure.Goal.L2,
            0.445,
            -0.047,
            0.16,
            0.028,
            Superstructure.Goal.L1
    );
    public static final SplineProfile L2_TO_L3 = new SplineProfile(
            0.74,
            Superstructure.Goal.L2,
            0.724,
            0.139,
            0.748,
            0.482,
            Superstructure.Goal.L3
    );
    public static final SplineProfile L2_TO_L4 = new SplineProfile(
            0.99,
            Superstructure.Goal.L2,
            0.724,
            0.139,
            0.858,
            0.400,
            Superstructure.Goal.L4
    );
    public static final SplineProfile L3_TO_L1 = new SplineProfile(
            0.75,
            Superstructure.Goal.L3,
            0.591,
            0,
            0.16,
            0.028,
            Superstructure.Goal.L1
    );
    public static final SplineProfile L3_TO_L2 = new SplineProfile(
            0.75,
            Superstructure.Goal.L3,
            0.748,
            0.482,
            0.724,
            0.139,
            Superstructure.Goal.L2
    );
    public static final SplineProfile L3_TO_L4 = new SplineProfile(
            0.67,
            Superstructure.Goal.L3,
            0.829,
            0.540,
            0.934,
            0.732,
            Superstructure.Goal.L4
    );
    public static final SplineProfile L4_TO_L1 = new SplineProfile(
            1.1,
            Superstructure.Goal.L4,
            0.922,
            0.261,
            0.579,
            0.075,
            Superstructure.Goal.L1
    );
    public static final SplineProfile L4_TO_L2 = new SplineProfile(
            1,
            Superstructure.Goal.L4,
            0.858,
            0.400,
            0.724,
            0.139,
            Superstructure.Goal.L2
    );
    public static final SplineProfile L4_TO_L3 = new SplineProfile(
            0.69,
            Superstructure.Goal.L4,
            0.934,
            0.732,
            0.829,
            0.540,
            Superstructure.Goal.L3
    );
}
