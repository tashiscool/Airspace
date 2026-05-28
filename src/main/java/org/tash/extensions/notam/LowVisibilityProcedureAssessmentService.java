package org.tash.extensions.notam;

import org.tash.extensions.weather.product.WeatherProduct;
import org.tash.extensions.weather.product.WeatherProductParseResult;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.Optional;

/**
 * Reconciles reported RVR, RVR equipment NOTAMs, and low-visibility procedure
 * terminology while deliberately avoiding any authoritative declaration about
 * live airport procedure state.
 */
public class LowVisibilityProcedureAssessmentService {
    public LowVisibilityProcedureAssessment assess(String airportId,
                                                   LowVisibilityProcedureProfile profile,
                                                   List<WeatherProductParseResult> weatherProducts,
                                                   List<DomesticNotamParseResult> domesticNotams) {
        String normalizedAirport = airportId == null ? null : airportId.trim().toUpperCase(Locale.US);
        List<String> sourceRefs = new ArrayList<>();
        List<String> reviewMessages = new ArrayList<>();
        List<String> diagnostics = new ArrayList<>();

        Optional<WeatherProduct> rvrWeather = safeWeather(weatherProducts).stream()
                .map(WeatherProductParseResult::getProduct)
                .filter(Objects::nonNull)
                .filter(product -> matchesAirport(product.getStationId(), normalizedAirport))
                .filter(product -> product.getRunwayVisualRangeFeet() != null)
                .max(Comparator.comparing(product -> product.getIssuedAt() == null ? product.getReceivedAt() : product.getIssuedAt(),
                        Comparator.nullsFirst(Comparator.naturalOrder())));
        Double reportedRvr = rvrWeather.map(WeatherProduct::getRunwayVisualRangeFeet).orElse(null);
        rvrWeather.ifPresent(product -> sourceRefs.add("WEATHER:" + product.getId()));

        DomesticNotamParseResult equipment = safeNotams(domesticNotams).stream()
                .filter(DomesticNotamParseResult::isAccepted)
                .filter(result -> isRvrEquipment(result) && matchesAirport(notamLocation(result), normalizedAirport))
                .findFirst()
                .orElse(null);
        DomesticNotamParseResult procedure = safeNotams(domesticNotams).stream()
                .filter(DomesticNotamParseResult::isAccepted)
                .filter(result -> isLowVisibilityProcedure(result) && matchesAirport(notamLocation(result), normalizedAirport))
                .findFirst()
                .orElse(null);

        String equipmentStatus = equipment == null ? null : statusLabel(equipment);
        String terminology = procedure == null ? null : lowVisibilityTerminologyLabel(procedure, profile);
        if (equipment != null) sourceRefs.add("NOTAM:" + notamRef(equipment));
        if (procedure != null) sourceRefs.add("NOTAM:" + notamRef(procedure));

        boolean belowProfileThreshold = reportedRvr != null
                && profile != null
                && profile.getAdvisoryRvrThresholdFeet() != null
                && reportedRvr <= profile.getAdvisoryRvrThresholdFeet();
        boolean ambiguity = reportedRvr != null || equipment != null || procedure != null;

        if (ambiguity) {
            reviewMessages.add("ATIS/source weather: confirm current reported RVR and trend from official broadcast or digital source.");
            reviewMessages.add("Tower/airport ops: confirm active local low-visibility protections and runway-specific surface-movement status.");
            reviewMessages.add("Company minima: verify operator-specific takeoff, taxi, and dispatch limits for the reported RVR.");
            reviewMessages.add("Local airport procedures: reconcile FAA/SMGCS terminology with ICAO/operator LVO/LVP wording before release planning.");
        }
        if (profile == null) {
            diagnostics.add("No airport low-visibility procedure profile was supplied; assessment remains source-artifact-only.");
        } else {
            diagnostics.add("Profile " + value(profile.getAirportId(), normalizedAirport) + " "
                    + value(profile.getSourceVersion(), "unversioned") + " retained as reference data, not live state.");
        }
        if (belowProfileThreshold) {
            diagnostics.add("Reported RVR is at or below the profile advisory threshold; procedure-state confirmation is recommended.");
        }
        if (equipment != null && reportedRvr != null) {
            diagnostics.add("METAR/SPECI RVR and RVR equipment NOTAM are correlated but retained as separate artifacts.");
        }

        return LowVisibilityProcedureAssessment.builder()
                .airportId(normalizedAirport)
                .reportedRvrFeet(reportedRvr)
                .rvrEquipmentStatus(equipmentStatus)
                .lowVisibilityProcedureTerminology(terminology)
                .recommendedAction(ambiguity ? "DELAY" : "MONITOR")
                .actionSublabel(ambiguity ? "CONFIRM PROCEDURE STATE" : "MONITOR PROCEDURE SOURCES")
                .ambiguityDetected(ambiguity)
                .separateArtifactsRetained(reportedRvr != null && equipment != null)
                .sourceRefs(distinct(sourceRefs))
                .reviewMessages(reviewMessages)
                .diagnostics(diagnostics)
                .build();
    }

    private List<WeatherProductParseResult> safeWeather(List<WeatherProductParseResult> weatherProducts) {
        return weatherProducts == null ? List.of() : weatherProducts;
    }

    private List<DomesticNotamParseResult> safeNotams(List<DomesticNotamParseResult> domesticNotams) {
        return domesticNotams == null ? List.of() : domesticNotams;
    }

    private boolean isRvrEquipment(DomesticNotamParseResult result) {
        String rule = value(result.getReducerRuleId(), "");
        String condition = value(result.getSemanticCondition(), "");
        return rule.equals("DOM2.RWY.RVR") || rule.equals("DOM2.AD.RVR_ALL") || rule.equals("DOM2.SVC.RVR")
                || condition.startsWith("RVR") || condition.startsWith("RVRT");
    }

    private boolean isLowVisibilityProcedure(DomesticNotamParseResult result) {
        return "DOM2.SVC.LOW_VISIBILITY_PROCEDURE".equals(result.getReducerRuleId())
                || "LOW_VISIBILITY_PROCEDURE".equals(result.getSemanticCondition());
    }

    private String statusLabel(DomesticNotamParseResult result) {
        String condition = value(result.getSemanticCondition(), "RVR");
        String action = value(result.getSemanticAction(), "REVIEW");
        return condition + " " + action;
    }

    private String lowVisibilityTerminologyLabel(DomesticNotamParseResult result,
                                                 LowVisibilityProcedureProfile profile) {
        String text = result.getRecord() == null ? "" : value(result.getRecord().getText(), "");
        String local = profile == null ? null : profile.getLocalProcedureName();
        return value(local, "LOW-VISIBILITY PROCEDURE") + " terminology retained from " + text;
    }

    private boolean matchesAirport(String candidate, String airportId) {
        if (airportId == null || airportId.isEmpty()) return true;
        if (candidate == null || candidate.isBlank()) return false;
        String value = candidate.toUpperCase(Locale.US);
        return value.equals(airportId) || value.equals(airportId.replaceFirst("^K", ""));
    }

    private String notamLocation(DomesticNotamParseResult result) {
        return result.getRecord() == null ? null : result.getRecord().getLocation();
    }

    private String notamRef(DomesticNotamParseResult result) {
        if (result.getRecord() == null) return "domestic-notam";
        return value(result.getRecord().getNotamNumber(), result.getRecord().getAccountability() + "-" + result.getRecord().getLocation());
    }

    private List<String> distinct(List<String> values) {
        return values.stream().filter(Objects::nonNull).distinct().toList();
    }

    private String value(String actual, String fallback) {
        return actual == null || actual.isBlank() ? fallback : actual;
    }
}
