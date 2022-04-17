package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class ShooterDynamicDistance {
    public static final double SHORT_THRESHOLD = 80;
    public static final double MIDDLE_THRESHOLD = 165;

    private Range currentRange = Range.Unknown;

    public enum Range {
        Unknown, Short, Middle, Long
    }

    public ShooterDynamicDistance() {
    }

    private final double OFFSET = 3;

    public Range getCurrentShootingRange(double distanceInches) {
        final double now = Timer.getFPGATimestamp();
        Range candidateRange = currentRange;
        final Range range;
        
        // Identify base candidate
        if (distanceInches < 0) {
            candidateRange = Range.Unknown;
            currentRange = candidateRange;
            return candidateRange;
        }
        else if (distanceInches < SHORT_THRESHOLD) {
            candidateRange = Range.Short;

        } else if (distanceInches < MIDDLE_THRESHOLD) {
            candidateRange = Range.Middle;
        } else {
            candidateRange = Range.Long;
        }


        // Determine range to handle
        if (currentRange == candidateRange) {
            return currentRange;
        }
        else if (Range.Short == currentRange) {
            if (distanceInches > (SHORT_THRESHOLD + OFFSET)) {
                range = candidateRange;
                currentRange = range;
            } else {
                range = currentRange;
            }
        } else if (Range.Middle == currentRange) {
            if ((distanceInches < (MIDDLE_THRESHOLD - OFFSET)) || (distanceInches > (MIDDLE_THRESHOLD + OFFSET))) {
                range = candidateRange;
                currentRange = range;
            } else {
                range = currentRange;
            }
        } else if (Range.Long == currentRange) {
            if (distanceInches < (MIDDLE_THRESHOLD - OFFSET)) {
                range = candidateRange;
                currentRange = range;
            } else {
                range = currentRange;
            }
        } else if (Range.Unknown == currentRange) {
            range = candidateRange;
            currentRange = range;
        } else {
            range = currentRange;
        }

        // System.out.println("[SDD] (" + now + ") dist: " + distanceInches + " | current: " + currentRange + " | candidate: " + candidateRange + " | range: " + range);

        return range;
    }
}
