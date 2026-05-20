package org.tash.extensions.carf.altrv;

import org.tash.extensions.carf.refdata.CarfReferenceDataProvider;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class AltrvRouteGraphValidator {
    public AltrvRouteGraphValidation validate(AltrvMessage message,
                                              AltrvRouteGraph graph,
                                              CarfReferenceDataProvider referenceDataProvider) {
        List<String> diagnostics = new ArrayList<>();
        List<AltrvDiagnostic> typedDiagnostics = new ArrayList<>();
        if (message == null) {
            add(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.ERROR, null, null,
                    "No ALTRV message to validate", null);
            return validation(false, diagnostics, typedDiagnostics);
        }
        if (graph == null || graph.getGraph() == null || graph.getGraph().vertexSet().isEmpty()) {
            add(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.ERROR, null, null,
                    "ALTRV route graph has no vertices", null);
        }
        Set<String> seen = new HashSet<>();
        for (AltrvRouteGroup group : message.getRouteGroups()) {
            for (AltrvRoute route : group.getRoutes()) {
                if (route.getPoints().size() < 2) {
                    add(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.ERROR, route.getId(), null,
                            "Route " + route.getId() + " has fewer than two points", null);
                }
                int previousMinutes = -1;
                for (AltrvRoutePoint point : route.getPoints()) {
                    String key = point.getId() + "@" + point.getTimeOffset();
                    if (!seen.add(group.getId() + ":" + route.getId() + ":" + key)) {
                        add(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.ERROR, route.getId(), null,
                                "Duplicate route graph point " + key, point.getSourceSpan());
                    }
                    if (!point.isCoordinateLiteral() && referenceDataProvider != null
                            && !referenceDataProvider.resolveFixOrNavaid(point.getId()).isPresent()) {
                        add(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.WARNING, route.getId(), null,
                                "Unresolved fix/navaid " + point.getId(), point.getSourceSpan());
                    }
                    int minutes = elapsedMinutes(point.getTimeOffset());
                    if (previousMinutes >= 0 && minutes >= 0 && minutes < previousMinutes) {
                        add(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.ERROR, route.getId(), null,
                                "Elapsed time " + point.getTimeOffset()
                                        + " is smaller than preceding route time", point.getSourceSpan());
                    }
                    if (minutes >= 0) {
                        previousMinutes = minutes;
                    }
                }
                if (route.isCommon() && !hasEvent(message, AltrvRouteEventType.JOIN)
                        && !hasEvent(message, AltrvRouteEventType.ROUTE_COMMON)) {
                    add(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.ERROR, route.getId(), null,
                            "Common route requires a common-route join/merge event", null);
                }
                if (hasEvent(message, AltrvRouteEventType.JOIN_COMMON_ROUTE)
                        && !hasEvent(message, AltrvRouteEventType.ROUTE_COMMON)
                        && !route.isCommon()) {
                    add(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.WARNING, route.getId(), null,
                            "JOIN CMN RTE found without an explicit common route declaration", null);
                }
                if (route.isBranch() && !hasEvent(message, AltrvRouteEventType.LEAVE)
                        && !hasEvent(message, AltrvRouteEventType.ROUTE_BRANCH)) {
                    add(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.ERROR, route.getId(), null,
                            "Branch route requires a leave/branch event", null);
                }
                if (hasEvent(message, AltrvRouteEventType.BRANCH_CLOSE)
                        && !hasEvent(message, AltrvRouteEventType.BRANCH_MERGE)) {
                    add(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.WARNING, route.getId(), null,
                            "Branch close found without branch merge point", null);
                }
                if (hasEvent(message, AltrvRouteEventType.RECEIVER_COMMON_ROUTE)
                        && !hasEvent(message, AltrvRouteEventType.JOIN_COMMON_ROUTE)) {
                    add(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.WARNING, route.getId(), null,
                            "Receiver common route found without JOIN CMN RTE", null);
                }
                validateStateTransitions(route, diagnostics, typedDiagnostics);
            }
        }
        if (graph != null && graph.getVertices() != null && graph.getEdges() != null) {
            for (AltrvRouteGraphVertex vertex : graph.getVertices().values()) {
                if (vertex.getType() == AltrvRouteGraphVertexType.DESTINATION
                        && graph.getEdges().stream().noneMatch(edge -> vertex.getId().equals(edge.getTargetId()))) {
                    add(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.ERROR, vertex.getRouteId(), vertex.getEventId(),
                            "Destination is unreachable: " + vertex.getLabel(), vertex.getSourceSpan());
                }
            }
        }
        validateDestinationCallsignFlow(message, diagnostics, typedDiagnostics);
        boolean hasHardError = diagnostics.stream().anyMatch(d ->
                d.startsWith("No ALTRV") || d.contains("fewer than two") || d.startsWith("ALTRV route graph"));
        boolean hasTypedError = typedDiagnostics.stream().anyMatch(d -> d.getSeverity() == AltrvDiagnosticSeverity.ERROR);
        return validation(!hasHardError && !hasTypedError, diagnostics, typedDiagnostics);
    }

    private void validateStateTransitions(AltrvRoute route,
                                          List<String> diagnostics,
                                          List<AltrvDiagnostic> typedDiagnostics) {
        boolean inAirRefueling = false;
        boolean inSupersonic = false;
        boolean altitudeChanging = false;
        for (AltrvRouteEvent event : route.getEvents()) {
            if (event.getType() == AltrvRouteEventType.CLIMB || event.getType() == AltrvRouteEventType.DESCEND) {
                altitudeChanging = true;
            }
            if ((event.getType() == AltrvRouteEventType.LVLOF_BY
                    || event.getType() == AltrvRouteEventType.LVLOF_WITHIN
                    || event.getType() == AltrvRouteEventType.LEVEL_OFF) && !altitudeChanging) {
                add(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.WARNING, route.getId(), event.getEventId(),
                        "Level-off event found without preceding climb/descend state", event.getSourceSpan());
            }
            if (event.getType() == AltrvRouteEventType.LVLOF_BY
                    || event.getType() == AltrvRouteEventType.LVLOF_WITHIN
                    || event.getType() == AltrvRouteEventType.LEVEL_OFF) {
                altitudeChanging = false;
            }
            if (event.getType() == AltrvRouteEventType.AIR_REFUELING_BEGIN) {
                inAirRefueling = true;
            }
            if (event.getType() == AltrvRouteEventType.AIR_REFUELING_END && !inAirRefueling) {
                add(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.ERROR, route.getId(), event.getEventId(),
                        "AIRFL ENDS without AIRFL BEGINS", event.getSourceSpan());
            }
            if (event.getType() == AltrvRouteEventType.AIR_REFUELING_END) {
                inAirRefueling = false;
            }
            if (event.getType() == AltrvRouteEventType.SUPERSONIC) {
                inSupersonic = !inSupersonic;
            }
        }
        if (inAirRefueling) {
            add(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.ERROR, route.getId(), null,
                    "Route ends while in air refueling", null);
        }
        if (inSupersonic) {
            add(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.WARNING, route.getId(), null,
                    "Route ends while supersonic state is active", null);
        }
    }

    private void validateDestinationCallsignFlow(AltrvMessage message,
                                                 List<String> diagnostics,
                                                 List<AltrvDiagnostic> typedDiagnostics) {
        Set<String> allowed = new HashSet<>();
        for (AltrvCallsign callsign : message.getCallsigns() == null ? Collections.<AltrvCallsign>emptyList() : message.getCallsigns()) {
            allowed.add(callsign.compact());
        }
        if (allowed.isEmpty()) {
            return;
        }
        for (AltrvDestination destination : message.getDestinations() == null ? Collections.<AltrvDestination>emptyList() : message.getDestinations()) {
            for (AltrvCallsign callsign : destination.getCallsigns() == null ? Collections.<AltrvCallsign>emptyList() : destination.getCallsigns()) {
                if (!allowed.contains(callsign.compact())) {
                    add(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.ERROR, null, null,
                            "Destination callsign " + callsign.compact() + " is not present in A section",
                            destination.getSourceSpan());
                }
            }
        }
    }

    private boolean hasEvent(AltrvMessage message, AltrvRouteEventType type) {
        return message.getEvents() != null && message.getEvents().stream().anyMatch(event -> event.getType() == type);
    }

    private int elapsedMinutes(String timeOffset) {
        if (timeOffset == null || !timeOffset.matches("\\d{4}")) {
            return -1;
        }
        return Integer.parseInt(timeOffset.substring(0, 2)) * 60 + Integer.parseInt(timeOffset.substring(2, 4));
    }

    private void add(List<String> diagnostics,
                     List<AltrvDiagnostic> typedDiagnostics,
                     AltrvDiagnosticSeverity severity,
                     String routeId,
                     String eventId,
                     String message,
                     AltrvSourceSpan sourceSpan) {
        diagnostics.add(message);
        typedDiagnostics.add(AltrvDiagnostic.builder()
                .severity(severity)
                .routeId(routeId)
                .eventId(eventId)
                .message(message)
                .sourceSpan(sourceSpan)
                .build());
    }

    private AltrvRouteGraphValidation validation(boolean valid,
                                                 List<String> diagnostics,
                                                 List<AltrvDiagnostic> typedDiagnostics) {
        return AltrvRouteGraphValidation.builder()
                .valid(valid)
                .diagnostics(Collections.unmodifiableList(new ArrayList<>(diagnostics)))
                .typedDiagnostics(Collections.unmodifiableList(new ArrayList<>(typedDiagnostics)))
                .build();
    }
}
