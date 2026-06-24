package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class GroundStopModel {
    private String groundStopId;
    private String scope;
    private String airportId;
    private String airspaceId;
    private String equipmentCriteria;
    private String reason;
    private int startOffsetMinutes;
    private int endOffsetMinutes;
    private boolean partial;
    @Builder.Default
    private List<String> affectedFlightIds = new ArrayList<>();
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
