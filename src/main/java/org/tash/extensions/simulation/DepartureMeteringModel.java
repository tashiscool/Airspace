package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class DepartureMeteringModel {
    private String meteringId;
    private String commonPoint;
    private int intervalSeconds;
    private int releaseRatePerHour;
    private int startOffsetMinutes;
    private int endOffsetMinutes;
    @Builder.Default
    private List<String> departureAirports = new ArrayList<>();
    @Builder.Default
    private List<String> affectedFlightIds = new ArrayList<>();
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
