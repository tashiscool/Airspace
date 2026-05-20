package org.tash.extensions.weather.pirep;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.decision.WeatherDecisionSeverity;

import java.time.Clock;
import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

public class PirepIngestService {
    private static final Duration DEFAULT_MAX_AGE = Duration.ofHours(2);

    private final Clock clock;
    private final Duration maxAge;
    private final PirepDuplicatePolicy duplicatePolicy;

    public PirepIngestService() {
        this(Clock.systemUTC(), DEFAULT_MAX_AGE);
    }

    public PirepIngestService(Clock clock, Duration maxAge) {
        this(clock, maxAge, PirepDuplicatePolicy.builder().build());
    }

    public PirepIngestService(Clock clock, Duration maxAge, PirepDuplicatePolicy duplicatePolicy) {
        this.clock = clock == null ? Clock.systemUTC() : clock;
        this.maxAge = maxAge == null ? DEFAULT_MAX_AGE : maxAge;
        this.duplicatePolicy = duplicatePolicy == null ? PirepDuplicatePolicy.builder().build() : duplicatePolicy;
    }

    public AutomatedPirepDraft automatedDraft(String aircraftId,
                                              String aircraftType,
                                              ZonedDateTime capturedAt,
                                              GeoCoordinate location) {
        return AutomatedPirepDraft.builder()
                .aircraftId(aircraftId)
                .aircraftType(aircraftType)
                .capturedAt(capturedAt)
                .location(location)
                .altitudeFeet(location == null ? 0.0 : location.getAltitude())
                .build();
    }

    public PirepValidationResult validate(PirepReport report) {
        List<String> errors = new ArrayList<>();
        List<String> warnings = new ArrayList<>();
        List<PirepDiagnostic> diagnostics = new ArrayList<>();
        if (report == null) {
            errors.add("PIREP is required");
            diagnostics.add(diagnostic(PirepDiagnosticType.INCOMPLETE, WeatherDecisionSeverity.WARNING, "PIREP is required"));
            return PirepValidationResult.builder().accepted(false).errors(errors).warnings(warnings).diagnostics(diagnostics).build();
        }
        if (isBlank(report.getAircraftType())) {
            errors.add("Aircraft type is required");
            diagnostics.add(diagnostic(PirepDiagnosticType.MISSING_AIRCRAFT_TYPE, WeatherDecisionSeverity.WARNING,
                    "Aircraft type is required"));
        }
        if (report.getObservationTime() == null) {
            errors.add("Observation time is required");
            diagnostics.add(diagnostic(PirepDiagnosticType.MISSING_OBSERVATION_TIME, WeatherDecisionSeverity.WARNING,
                    "Observation time is required"));
        } else {
            ZonedDateTime now = ZonedDateTime.now(clock);
            if (report.getObservationTime().isAfter(now.plusMinutes(5))) {
                errors.add("Observation time cannot be in the future");
                diagnostics.add(diagnostic(PirepDiagnosticType.FUTURE, WeatherDecisionSeverity.WARNING,
                        "Observation time cannot be in the future"));
            }
            if (report.getObservationTime().plus(maxAge).isBefore(now)) {
                warnings.add("PIREP observation is older than " + maxAge.toMinutes() + " minutes");
                diagnostics.add(diagnostic(PirepDiagnosticType.STALE, WeatherDecisionSeverity.ADVISORY,
                        "PIREP observation is older than " + maxAge.toMinutes() + " minutes"));
            }
        }
        if (report.getLocation() == null) {
            errors.add("Location is required");
            diagnostics.add(diagnostic(PirepDiagnosticType.MISSING_LOCATION, WeatherDecisionSeverity.WARNING,
                    "Location is required"));
            if (!isBlank(report.getLocationText())) {
                warnings.add("PIREP location text requires reference-data resolution: " + report.getLocationText());
                diagnostics.add(diagnostic(PirepDiagnosticType.UNRESOLVED_LOCATION_TEXT, WeatherDecisionSeverity.ADVISORY,
                        "PIREP location text requires reference-data resolution: " + report.getLocationText()));
            }
        } else if (report.getLocationQuality() != null && report.getLocationQuality() < 0.5) {
            warnings.add("PIREP location quality is low");
            diagnostics.add(diagnostic(PirepDiagnosticType.LOW_QUALITY_LOCATION, WeatherDecisionSeverity.ADVISORY,
                    "PIREP location quality is low"));
        }
        if (report.getAltitudeFeet() == null) {
            errors.add("Altitude is required");
            diagnostics.add(diagnostic(PirepDiagnosticType.MISSING_ALTITUDE, WeatherDecisionSeverity.WARNING,
                    "Altitude is required"));
        }
        if (report.getPhenomenon() == null) {
            errors.add("Weather phenomenon is required");
            diagnostics.add(diagnostic(PirepDiagnosticType.MISSING_PHENOMENON, WeatherDecisionSeverity.WARNING,
                    "Weather phenomenon is required"));
        }
        if (report.getIntensity() == null || report.getIntensity() == PirepIntensity.UNKNOWN) {
            warnings.add("PIREP intensity is unknown");
            diagnostics.add(diagnostic(PirepDiagnosticType.UNKNOWN_INTENSITY, WeatherDecisionSeverity.ADVISORY,
                    "PIREP intensity is unknown"));
        }
        if (report.getCodingQuality() != null && report.getCodingQuality() < 0.5) {
            warnings.add("PIREP coding quality is low");
            diagnostics.add(diagnostic(PirepDiagnosticType.MISCODED, WeatherDecisionSeverity.ADVISORY,
                    "PIREP coding quality is low"));
        }
        return PirepValidationResult.builder()
                .accepted(errors.isEmpty())
                .errors(errors)
                .warnings(warnings)
                .diagnostics(diagnostics)
                .build();
    }

    public PirepIngestResult ingest(PirepReport report, PirepRepositoryView repositoryView) {
        PirepValidationResult validation = validate(report);
        List<PirepDiagnostic> diagnostics = new ArrayList<>(validation.getDiagnostics());
        if (repositoryView != null && report != null) {
            for (PirepReport existing : repositoryView.recentReports()) {
                if (duplicatePolicy.isDuplicate(report, existing)) {
                    diagnostics.add(diagnostic(PirepDiagnosticType.DUPLICATE, WeatherDecisionSeverity.ADVISORY,
                            "PIREP duplicates an existing report"));
                    return PirepIngestResult.builder()
                            .accepted(false)
                            .report(report)
                            .disseminationStatus(PirepDisseminationStatus.DUPLICATE_SUPPRESSED)
                            .diagnostics(diagnostics)
                            .build();
                }
            }
        }
        return PirepIngestResult.builder()
                .accepted(validation.isAccepted())
                .report(report)
                .disseminationStatus(validation.isAccepted()
                        ? PirepDisseminationStatus.READY
                        : PirepDisseminationStatus.REJECTED)
                .diagnostics(diagnostics)
                .build();
    }

    private boolean isBlank(String value) {
        return value == null || value.trim().isEmpty();
    }

    private PirepDiagnostic diagnostic(PirepDiagnosticType type, WeatherDecisionSeverity severity, String message) {
        return PirepDiagnostic.builder()
                .type(type)
                .severity(severity)
                .message(message)
                .build();
    }
}
