package org.tash.extensions.carf.altrv;

import java.time.Duration;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class AltrvMessageValidator {
    public List<String> validate(AltrvMessage message) {
        List<String> diagnostics = new ArrayList<>();
        if (message == null) {
            diagnostics.add("No ALTRV message supplied for validation");
            return diagnostics;
        }
        validateCallsigns(message.getCallsigns(), diagnostics);
        validateAircraftCounts(message, diagnostics);
        validateDepartures(message, diagnostics);
        validateAvana(message, diagnostics);
        validateStationary(message, diagnostics);
        return Collections.unmodifiableList(diagnostics);
    }

    private void validateCallsigns(List<AltrvCallsign> callsigns, List<String> diagnostics) {
        Set<String> seen = new HashSet<>();
        for (AltrvCallsign callsign : empty(callsigns)) {
            String compact = callsign.compact();
            if (compact.isEmpty()) {
                diagnostics.add("Blank callsign in A section");
            }
            if (compact.length() > 7) {
                diagnostics.add("Callsign exceeds seven combined characters: " + compact);
            }
            if (!seen.add(compact)) {
                diagnostics.add("Duplicate callsign detected: " + compact);
            }
        }
    }

    private void validateAircraftCounts(AltrvMessage message, List<String> diagnostics) {
        int aircraftCount = 0;
        for (AltrvAircraftType type : empty(message.getAircraftTypes())) {
            aircraftCount += Math.max(0, type.getCount());
        }
        if (!empty(message.getAircraftTypes()).isEmpty() && aircraftCount != empty(message.getCallsigns()).size()) {
            diagnostics.add("Number of callsigns (" + empty(message.getCallsigns()).size()
                    + ") is different than number of aircraft types (" + aircraftCount + ")");
        }
    }

    private void validateDepartures(AltrvMessage message, List<String> diagnostics) {
        List<AltrvCallsign> fCallsigns = new ArrayList<>();
        for (AltrvDepartureGroup group : empty(message.getDepartureGroups())) {
            fCallsigns.addAll(empty(group.getCallsigns()));
        }
        if (!fCallsigns.isEmpty() && !sameCallsigns(message.getCallsigns(), fCallsigns)) {
            diagnostics.add("Callsigns in A section do not match callsigns in F section");
        }
        if (empty(message.getCallsigns()).size() > 1 && !message.hasEvent(AltrvRouteEventType.ADMIS_MITO)
                && !message.hasEvent(AltrvRouteEventType.ADMIS_SECONDS)) {
            diagnostics.add("Multi-aircraft departure groups require ADMIS");
        }
    }

    private void validateAvana(AltrvMessage message, List<String> diagnostics) {
        if (message.getFirstDepartureTime() == null || message.getAvanaTime() == null) {
            return;
        }
        long minutes = Duration.between(message.getFirstDepartureTime(), message.getAvanaTime()).toMinutes();
        if (minutes < 5 || minutes > 24 * 60) {
            diagnostics.add("AVANA must be 5 minutes to 24 hours after first departure");
        }
    }

    private void validateStationary(AltrvMessage message, List<String> diagnostics) {
        if (message.getStationaryReservation() != null && message.getTimingText() != null
                && !message.getTimingText().trim().isEmpty()) {
            diagnostics.add("Stationary reservations cannot have F departures");
        }
    }

    private boolean sameCallsigns(List<AltrvCallsign> first, List<AltrvCallsign> second) {
        Set<String> a = new HashSet<>();
        for (AltrvCallsign callsign : empty(first)) {
            a.add(callsign.compact());
        }
        Set<String> b = new HashSet<>();
        for (AltrvCallsign callsign : empty(second)) {
            b.add(callsign.compact());
        }
        return a.equals(b);
    }

    private <T> List<T> empty(List<T> values) {
        return values == null ? Collections.emptyList() : values;
    }
}
