package frc.robot.auto;

import choreo.auto.AutoRoutine;
import frc.robot.constants.Constants;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public record AutoOption(
        String name,
        AutoRoutine autoRoutine,
        Set<Constants.CompetitionType> competitionTypes
) {
    public static final List<Constants.CompetitionType> defaultCompetitionTypes =
            List.of(Constants.CompetitionType.TESTING);

    public AutoOption(final String name, final AutoRoutine autoRoutine) {
        this(name, autoRoutine, new HashSet<>(defaultCompetitionTypes));
    }

    public AutoOption(
            final String name,
            final AutoRoutine autoRoutine,
            final Constants.CompetitionType... competitionTypes
    ) {
        this(
                name,
                autoRoutine,
                addDefaultCompetitionType(competitionTypes)
        );
    }

    public boolean hasCompetitionType(final Constants.CompetitionType competitionType) {
        return competitionTypes.contains(competitionType);
    }

    private static Set<Constants.CompetitionType> addDefaultCompetitionType(
            final Constants.CompetitionType[] competitionTypes
    ) {
        final HashSet<Constants.CompetitionType> set = new HashSet<>(defaultCompetitionTypes);
        set.addAll(Arrays.asList(competitionTypes));
        return set;
    }
}
