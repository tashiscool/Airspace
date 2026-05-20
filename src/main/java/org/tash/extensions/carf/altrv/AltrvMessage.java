package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;

import java.util.List;
import java.util.Map;
import java.time.ZonedDateTime;

@Data
@Builder
public class AltrvMessage {
    private String rawText;
    private Map<String, String> sections;
    private String activityName;
    private String mission;
    private String location;
    private String routeText;
    private String timingText;
    private String aircraftText;
    private List<AltrvCallsign> callsigns;
    private List<AltrvAircraftType> aircraftTypes;
    private List<AltrvDepartureGroup> departureGroups;
    private List<AltrvArea> areas;
    private List<AltrvDestination> destinations;
    private List<AltrvExit> exits;
    private List<AltrvSectionResult> sectionResults;
    private AltrvStationaryReservation stationaryReservation;
    private ZonedDateTime firstDepartureTime;
    private ZonedDateTime avanaTime;
    private List<AltrvRouteGroup> routeGroups;
    private List<AltrvRouteEvent> events;
    private List<String> diagnostics;

    public boolean hasEvent(AltrvRouteEventType type) {
        return events != null && events.stream().anyMatch(event -> event.getType() == type);
    }
}
