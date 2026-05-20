package org.tash.extensions.carf.api;

import org.tash.extensions.carf.altrv.AltrvMessage;
import org.tash.extensions.carf.altrv.AltrvParseResult;
import org.tash.extensions.carf.altrv.AltrvParser;
import org.tash.extensions.carf.altrv.AltrvReservationMapper;
import org.tash.extensions.carf.altrv.AltrvRouteGraph;
import org.tash.extensions.carf.altrv.AltrvRouteGraphBuilder;
import org.tash.extensions.carf.altrv.AltrvRouteGraphValidation;
import org.tash.extensions.carf.altrv.AltrvRouteGraphValidator;
import org.tash.extensions.carf.altrv.AltrvSpatialMapper;
import org.tash.extensions.carf.refdata.CarfReferenceDataProvider;
import org.tash.extensions.carf.refdata.DefaultCarfReferenceDataProvider;
import org.tash.extensions.reservation.AirspaceReservation;
import org.tash.extensions.reservation.CarfRouteMessage;
import org.tash.extensions.reservation.CarfRouteMessageParser;
import org.tash.extensions.reservation.ReservationConflict;
import org.tash.extensions.reservation.ReservationConflictDetector;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class CarfAnalysisService {
    private final AltrvParser altrvParser;
    private final AltrvReservationMapper reservationMapper;
    private final CarfReferenceDataProvider referenceDataProvider;
    private final ReservationConflictDetector conflictDetector;

    public CarfAnalysisService() {
        this(new AltrvParser(), new AltrvReservationMapper(), new DefaultCarfReferenceDataProvider(),
                new ReservationConflictDetector());
    }

    public CarfAnalysisService(AltrvParser altrvParser,
                               AltrvReservationMapper reservationMapper,
                               CarfReferenceDataProvider referenceDataProvider,
                               ReservationConflictDetector conflictDetector) {
        this.altrvParser = altrvParser;
        this.reservationMapper = reservationMapper;
        this.referenceDataProvider = referenceDataProvider;
        this.conflictDetector = conflictDetector;
    }

    public CarfAnalysisResult parse(String raw) {
        return parseAndMap(raw);
    }

    public CarfAnalysisResult parseAndMap(String raw) {
        return parseValidateMap(raw);
    }

    public CarfAnalysisResult parseValidateMap(String raw) {
        List<String> diagnostics = new ArrayList<>();
        List<String> blocked = new ArrayList<>();
        List<String> unsupported = new ArrayList<>();
        AltrvParseResult parseResult = altrvParser.parse(raw);
        diagnostics.addAll(parseResult.getDiagnostics());
        if (!parseResult.isAccepted()) {
            return result(false, parseResult, null, Collections.emptyList(), Collections.emptyList(),
                    diagnostics, unsupported, blocked);
        }

        AltrvMessage message = parseResult.getMessage();
        AltrvRouteGraph graph = new AltrvRouteGraphBuilder().build(message);
        AltrvRouteGraphValidation graphValidation =
                new AltrvRouteGraphValidator().validate(message, graph, referenceDataProvider);
        diagnostics.addAll(graphValidation.getDiagnostics());

        try {
            List<AirspaceReservation> reservations = reservationMapper.toReservations(message, referenceDataProvider);
            List<AirspaceReservation> spatialReservations = new ArrayList<>(new AltrvSpatialMapper()
                    .toReservations(value(message.getActivityName()), message, referenceDataProvider));
            spatialReservations.removeIf(reservation -> "ROUTE_SEGMENT".equals(reservation.getReservationType()));
            if (!spatialReservations.isEmpty()) {
                reservations = new ArrayList<>(reservations);
                reservations.addAll(spatialReservations);
            }
            CarfRouteMessage routeMessage = new CarfRouteMessageParser(name -> referenceDataProvider.resolveFixOrNavaid(name))
                    .parse(raw);
            return result(graphValidation.isValid(), parseResult, routeMessage, reservations,
                    Collections.emptyList(), diagnostics, unsupported, blocked);
        } catch (RuntimeException ex) {
            diagnostics.add("ALTRV reservation mapping failed: " + ex.getMessage());
            return result(false, parseResult, null, Collections.emptyList(), Collections.emptyList(),
                    diagnostics, unsupported, blocked);
        }
    }

    public CarfAnalysisResult analyzeConflicts(Collection<AirspaceReservation> reservations) {
        List<AirspaceReservation> list = reservations == null
                ? Collections.emptyList()
                : new ArrayList<>(reservations);
        List<ReservationConflict> conflicts = new ArrayList<>();
        for (int i = 0; i < list.size(); i++) {
            for (int j = i + 1; j < list.size(); j++) {
                ReservationConflict conflict = conflictDetector.detectConflict(list.get(i), list.get(j));
                if (conflict != null) {
                    conflicts.add(conflict);
                }
            }
        }
        return result(true, null, null, list, conflicts, Collections.emptyList(),
                Collections.emptyList(), Collections.emptyList());
    }

    private CarfAnalysisResult result(boolean accepted,
                                      AltrvParseResult parseResult,
                                      CarfRouteMessage routeMessage,
                                      List<AirspaceReservation> reservations,
                                      List<ReservationConflict> conflicts,
                                      List<String> diagnostics,
                                      List<String> unsupported,
                                      List<String> blocked) {
        Map<String, String> metadata = new LinkedHashMap<>();
        if (parseResult != null && parseResult.getMessage() != null) {
            metadata.put("activityName", value(parseResult.getMessage().getActivityName()));
            metadata.put("mission", value(parseResult.getMessage().getMission()));
            metadata.put("location", value(parseResult.getMessage().getLocation()));
        }
        return CarfAnalysisResult.builder()
                .accepted(accepted)
                .altrvParseResult(parseResult)
                .routeMessage(routeMessage)
                .reservations(reservations)
                .conflicts(conflicts)
                .diagnostics(diagnostics)
                .unsupportedNotes(unsupported)
                .blockedNotes(blocked)
                .sourceMetadata(metadata)
                .build();
    }

    private String value(String value) {
        return value == null ? "" : value;
    }
}
