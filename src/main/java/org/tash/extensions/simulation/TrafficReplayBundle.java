package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class TrafficReplayBundle {
    private String id;
    private String sourceId;
    private String sourceMode;
    private String providerFamily;
    private String generatedAt;
    private String timeBasis;
    private String providerReceiptId;
    private String authorizationMode;
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
    @Builder.Default
    private List<TrafficReplayFlightPlan> flightPlans = new ArrayList<>();
    @Builder.Default
    private List<TrafficReplayPosition> positions = new ArrayList<>();
    @Builder.Default
    private List<TrafficReplayAirportDemand> airportDemand = new ArrayList<>();
    @Builder.Default
    private List<TrafficReplaySectorDemand> sectorDemand = new ArrayList<>();
    @Builder.Default
    private List<TrafficManagementInitiative> trafficManagementInitiatives = new ArrayList<>();
    @Builder.Default
    private List<String> assumptions = new ArrayList<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
