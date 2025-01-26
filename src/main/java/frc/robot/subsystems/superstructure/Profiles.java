package frc.robot.subsystems.superstructure;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

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

    //starting goal -> ending goal -> profile
    private static final Map<Superstructure.Goal, Map<Superstructure.Goal, SplineProfile>> profiles = Map.of(
            Superstructure.Goal.L1, Map.of(
                    Superstructure.Goal.L2, L1_TO_L2,
                    Superstructure.Goal.L3, L1_TO_L3,
                    Superstructure.Goal.L4, L1_TO_L4
            ),
            Superstructure.Goal.L2, Map.of(
                    Superstructure.Goal.L1, L2_TO_L1,
                    Superstructure.Goal.L3, L2_TO_L3,
                    Superstructure.Goal.L4, L2_TO_L4
            ),
            Superstructure.Goal.L3, Map.of(
                    Superstructure.Goal.L1, L3_TO_L1,
                    Superstructure.Goal.L2, L3_TO_L2,
                    Superstructure.Goal.L4, L3_TO_L4
            ),
            Superstructure.Goal.L4, Map.of(
                    Superstructure.Goal.L1, L4_TO_L1,
                    Superstructure.Goal.L2, L4_TO_L2,
                    Superstructure.Goal.L3, L4_TO_L3
            )
    );

    public static Optional<SplineProfile> getProfile(Superstructure.Goal start, Superstructure.Goal end) {
        final Map<Superstructure.Goal, SplineProfile> startProfiles = profiles.get(start);
        if (startProfiles == null) {
            return Optional.empty();
        }

        return Optional.ofNullable(startProfiles.get(end));
    }
}
