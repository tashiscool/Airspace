package org.tash.extensions.reservation;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.time.Duration;
import java.time.ZonedDateTime;

/**
 * Extracted from the 2010 draft "Protected Airspace Rules for CARF" document.
 */
public class CarfProtectedAirspaceRules {
    public boolean isSeparated(boolean verticalSeparationMet,
                               boolean lateralSeparationMet,
                               boolean longitudinalSeparationMet) {
        return verticalSeparationMet || lateralSeparationMet || longitudinalSeparationMet;
    }

    public double deconflictionLowerAltitudeFeet(double lowerAltitudeFeet, double verticalSeparationFeet) {
        return lowerAltitudeFeet - verticalSeparationFeet / 2.0;
    }

    public double deconflictionUpperAltitudeFeet(double upperAltitudeFeet, double verticalSeparationFeet) {
        return upperAltitudeFeet + verticalSeparationFeet / 2.0;
    }

    public double requiredLongitudinalMinutes(double firstStandardMinutes, double secondStandardMinutes) {
        return Math.max(firstStandardMinutes, secondStandardMinutes);
    }

    public boolean verticalSeparationEstablishedForPassage(ZonedDateTime levelOffTime,
                                                           ZonedDateTime estimatedPassageTime,
                                                           double requiredMinutes) {
        return !levelOffTime.isAfter(estimatedPassageTime.minusMinutes((long) requiredMinutes));
    }

    public double minutesVerticalSeparationEstablishedBeforePassage(ZonedDateTime levelOffTime,
                                                                    ZonedDateTime estimatedPassageTime) {
        return Duration.between(levelOffTime, estimatedPassageTime).toMillis() / 60000.0;
    }

    public boolean longitudinalTimeWindowsSeparated(ZonedDateTime firstStart,
                                                    ZonedDateTime firstEnd,
                                                    ZonedDateTime secondStart,
                                                    ZonedDateTime secondEnd,
                                                    double requiredMinutes) {
        ZonedDateTime firstBufferedStart = firstStart.minusMinutes((long) requiredMinutes);
        ZonedDateTime firstBufferedEnd = firstEnd.plusMinutes((long) requiredMinutes);
        ZonedDateTime secondBufferedStart = secondStart.minusMinutes((long) requiredMinutes);
        ZonedDateTime secondBufferedEnd = secondEnd.plusMinutes((long) requiredMinutes);
        return firstBufferedEnd.isBefore(secondBufferedStart) || secondBufferedEnd.isBefore(firstBufferedStart);
    }

    public List<String> principles() {
        return Collections.unmodifiableList(Arrays.asList(
                "Vertical, lateral, and longitudinal criteria are independent.",
                "Aircraft are separated when at least one criterion is valid.",
                "Vertical protected airspace applies half the required separation below and above the block.",
                "Climb/descent protection reserves both ranges until the level-off event starts.",
                "Lateral route protection is a pill shape around the route segment.",
                "Longitudinal separation applies the larger standard between the aircraft.",
                "Opposite-course vertical separation must be established X minutes before through X minutes after passage."
        ));
    }
}
