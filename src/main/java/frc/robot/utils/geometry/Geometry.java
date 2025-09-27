package frc.robot.utils.geometry;

import edu.wpi.first.math.geometry.Translation2d;

public final class Geometry {
    private static boolean ccw(Translation2d A, Translation2d B, Translation2d C) {
        return (C.getY() - A.getY()) * (B.getX() - A.getX()) > (B.getY()- A.getY()) * (C.getX() - A.getX());
    }

    public static boolean checkSegmentInteference(
            final Translation2d line1Start,
            final Translation2d line1End,
            final Translation2d line2Start,
            final Translation2d line2End
    ) {
        return ccw(line1Start, line2Start, line2End) != ccw(line1End, line2Start, line2End)
                && ccw(line1Start, line1End, line2Start) != ccw(line1Start, line1End, line2End);
    }
}