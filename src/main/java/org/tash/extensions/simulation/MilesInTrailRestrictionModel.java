package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class MilesInTrailRestrictionModel {
    private String restrictionId;
    private int milesInTrail;
    private int minutesInTrail;
    private String fixId;
    private String sectorId;
    private String routeId;
    private String destination;
    private String altitudeStratum;
    private int startOffsetMinutes;
    private int endOffsetMinutes;
    @Builder.Default
    private List<String> affectedFlightIds = new ArrayList<>();
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
