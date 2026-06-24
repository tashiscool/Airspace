package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class TrafficManagementInitiative {
    private String id;
    private String type;
    private TrafficManagementInitiativeType primitiveType;
    private String status;
    private String scope;
    private String targetResourceId;
    private String reason;
    private String constraintId;
    private int startOffsetMinutes;
    private int endOffsetMinutes;
    private int expectedDelayMinutes;
    private double confidence;
    private FlowEvaluationAreaModel flowArea;
    private TrafficFlowProgramModel flowProgram;
    private MilesInTrailRestrictionModel milesInTrail;
    private RerouteAdvisoryModel rerouteAdvisory;
    private GroundStopModel groundStop;
    private DepartureMeteringModel departureMetering;
    private ArrivalRateModel arrivalRate;
    private SectorCapacityModel sectorCapacity;
    private TmiRecommendationModel recommendation;
    @Builder.Default
    private List<String> affectedFlightIds = new ArrayList<>();
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
