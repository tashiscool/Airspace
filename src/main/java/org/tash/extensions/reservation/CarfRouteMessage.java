package org.tash.extensions.reservation;

import lombok.Builder;
import lombok.Data;

import java.time.ZonedDateTime;
import java.util.List;

@Data
@Builder
public class CarfRouteMessage {
    private String activityName;
    private String mission;
    private String location;
    private String routeText;
    private ZonedDateTime estimatedDepartureTime;
    private ZonedDateTime avanaTime;
    private int admissionMinutes;
    private int admissionSeconds;
    private int trueAirspeedKnots;
    private String projectOfficer;
    private String artccsConcerned;
    private String additionalInfo;
    private List<String> resolvedWaypointNames;
    private List<String> unresolvedWaypointNames;
    private List<AirspaceReservation> reservations;
}
