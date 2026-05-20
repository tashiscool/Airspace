package org.tash.extensions.weather.pirep;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;

@Data
@Builder
public class AutomatedPirepDraft {
    private final String aircraftId;
    private final String aircraftType;
    private final ZonedDateTime capturedAt;
    private final GeoCoordinate location;
    private final double altitudeFeet;

    public PirepReport toReport(PirepPhenomenon phenomenon, PirepIntensity intensity, String remarks) {
        return PirepReport.builder()
                .id(aircraftId + "-" + capturedAt.toInstant().toEpochMilli())
                .aircraftId(aircraftId)
                .aircraftType(aircraftType)
                .observationTime(capturedAt)
                .receivedTime(capturedAt)
                .location(location)
                .altitudeFeet(altitudeFeet)
                .phenomenon(phenomenon)
                .intensity(intensity)
                .urgent(intensity == PirepIntensity.SEVERE || intensity == PirepIntensity.EXTREME)
                .remarks(remarks)
                .source("AUTOMATED_DRAFT")
                .normalizedText(remarks)
                .locationQuality(location == null ? 0.0 : 1.0)
                .codingQuality(phenomenon == null || intensity == null ? 0.5 : 1.0)
                .disseminationStatus(PirepDisseminationStatus.READY)
                .build();
    }
}
