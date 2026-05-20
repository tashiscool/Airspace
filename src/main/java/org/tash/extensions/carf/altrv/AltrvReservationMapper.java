package org.tash.extensions.carf.altrv;

import org.tash.extensions.carf.refdata.CarfReferenceDataProvider;
import org.tash.extensions.reservation.AirspaceReservation;
import org.tash.extensions.reservation.CarfRouteMessage;
import org.tash.extensions.reservation.CarfRouteMessageParser;
import org.tash.extensions.reservation.CarfWaypointResolver;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class AltrvReservationMapper {
    public List<AirspaceReservation> toReservations(AltrvMessage message, CarfReferenceDataProvider refs) {
        if (message == null) {
            return java.util.Collections.emptyList();
        }
        CarfRouteMessage routeMessage = new CarfRouteMessageParser(asWaypointResolver(refs)).parse(message.getRawText());
        AltrvRouteGraph graph = new AltrvRouteGraphBuilder().build(message);
        List<String> graphNodeIds = graph.getNodeOrder();
        List<String> diagnostics = new ArrayList<>();
        if (message.getDiagnostics() != null) {
            diagnostics.addAll(message.getDiagnostics());
        }
        List<String> eventNames = message.getEvents().stream()
                .map(event -> event.getType().name())
                .collect(Collectors.toList());
        for (AirspaceReservation reservation : routeMessage.getReservations()) {
            reservation.setRouteGraphNodeIds(graphNodeIds);
            reservation.setDiagnostics(diagnostics);
            if (!eventNames.isEmpty()) {
                reservation.setSourceText((reservation.getSourceText() == null ? "" : reservation.getSourceText())
                        + " events=" + eventNames);
            }
        }
        return routeMessage.getReservations();
    }

    private CarfWaypointResolver asWaypointResolver(CarfReferenceDataProvider refs) {
        if (refs == null) {
            return new org.tash.extensions.reservation.DefaultCarfWaypointResolver();
        }
        return name -> {
            Optional<org.tash.data.GeoCoordinate> resolved = refs.resolveFixOrNavaid(name);
            return resolved;
        };
    }
}
