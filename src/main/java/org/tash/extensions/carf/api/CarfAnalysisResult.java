package org.tash.extensions.carf.api;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.carf.altrv.AltrvParseResult;
import org.tash.extensions.reservation.AirspaceReservation;
import org.tash.extensions.reservation.CarfRouteMessage;
import org.tash.extensions.reservation.ReservationConflict;

import java.util.List;
import java.util.Map;

@Data
@Builder
public class CarfAnalysisResult {
    private boolean accepted;
    private AltrvParseResult altrvParseResult;
    private CarfRouteMessage routeMessage;
    private List<AirspaceReservation> reservations;
    private List<ReservationConflict> conflicts;
    private List<String> diagnostics;
    private List<String> unsupportedNotes;
    private List<String> blockedNotes;
    private Map<String, String> sourceMetadata;
}
