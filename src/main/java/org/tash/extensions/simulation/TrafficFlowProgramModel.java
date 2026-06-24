package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class TrafficFlowProgramModel {
    private String programId;
    private TrafficManagementInitiativeType programType;
    private String controlledResourceType;
    private String controlledResourceId;
    private String targetAirport;
    private String flowAreaId;
    private int startOffsetMinutes;
    private int endOffsetMinutes;
    private int acceptanceRatePerHour;
    private int arrivalRatePerHour;
    private int departureRatePerHour;
    private int edctWindowMinutes;
    private String delayAssignmentPolicy;
    @Builder.Default
    private List<String> affectedFlightIds = new ArrayList<>();
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
