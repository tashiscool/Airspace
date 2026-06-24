package org.tash.extensions.simulation;

import java.util.Locale;

public enum TrafficManagementInitiativeType {
    GDP,
    AFP,
    FEA,
    FCA,
    MILES_IN_TRAIL,
    MINUTES_IN_TRAIL,
    REROUTE_ADVISORY,
    REQUIRED_REROUTE,
    GROUND_STOP,
    DEPARTURE_METERING,
    ARRIVAL_RATE,
    SECTOR_CAPACITY,
    CTOP,
    LEVEL_CAPPING,
    FIX_BALANCING,
    AIRBORNE_HOLDING,
    LOCAL_REVIEW,
    UNKNOWN;

    public static TrafficManagementInitiativeType fromCode(String value) {
        if (value == null || value.isBlank()) return UNKNOWN;
        String normalized = value.trim().toUpperCase(Locale.US)
                .replace('-', '_')
                .replace(' ', '_')
                .replace("/", "_");
        return switch (normalized) {
            case "GDP", "GROUND_DELAY_PROGRAM" -> GDP;
            case "AFP", "AIRSPACE_FLOW_PROGRAM" -> AFP;
            case "FEA", "FLOW_EVALUATION_AREA" -> FEA;
            case "FCA", "FLOW_CONSTRAINED_AREA", "FLOW_CONTROL_AREA" -> FCA;
            case "MIT", "MILES_IN_TRAIL", "MILESINTRAIL" -> MILES_IN_TRAIL;
            case "MINIT", "MINUTES_IN_TRAIL", "MINUTESINTRAIL" -> MINUTES_IN_TRAIL;
            case "REROUTE", "REROUTE_ADVISORY", "PDRR", "ABRR" -> REROUTE_ADVISORY;
            case "RR", "REQUIRED_REROUTE" -> REQUIRED_REROUTE;
            case "GS", "GROUND_STOP", "GROUND_STOPS" -> GROUND_STOP;
            case "DSP", "MDI", "DEPARTURE_METERING", "DEPARTURE_SEQUENCING_PROGRAM", "MINIMUM_DEPARTURE_INTERVAL" -> DEPARTURE_METERING;
            case "AAR", "ARRIVAL_RATE", "AIRPORT_ACCEPTANCE_RATE" -> ARRIVAL_RATE;
            case "SECTOR_CAPACITY", "SECTOR_RATE", "ACCEPTANCE_RATE" -> SECTOR_CAPACITY;
            case "CTOP", "COLLABORATIVE_TRAJECTORY_OPTIONS_PROGRAM" -> CTOP;
            case "LEVEL_CAPPING", "LVLCP", "CAPPING", "TUNNELING" -> LEVEL_CAPPING;
            case "FIX_BALANCING", "FXBAL" -> FIX_BALANCING;
            case "AIRBORNE_HOLDING", "ABHLD" -> AIRBORNE_HOLDING;
            case "REROUTE_REVIEW", "LOCAL_REVIEW", "TMI_REVIEW" -> LOCAL_REVIEW;
            default -> UNKNOWN;
        };
    }
}
