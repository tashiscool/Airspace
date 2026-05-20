package org.tash.extensions.weather.coordination;

import org.tash.extensions.weather.decision.RouteBlockagePrediction;
import org.tash.extensions.weather.decision.RouteWeatherAdvisory;
import org.tash.extensions.weather.decision.WeatherDecisionAction;
import org.tash.extensions.weather.decision.WeatherDecisionSeverity;
import org.tash.extensions.weather.decision.WeatherRecommendedAction;
import org.tash.extensions.weather.pirep.PirepDiagnostic;
import org.tash.extensions.weather.pirep.PirepDiagnosticType;
import org.tash.extensions.weather.pirep.PirepIngestResult;

import java.time.Clock;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class WeatherCoordinationService {
    private final Clock clock;

    public WeatherCoordinationService() {
        this(Clock.systemUTC());
    }

    public WeatherCoordinationService(Clock clock) {
        this.clock = clock == null ? Clock.systemUTC() : clock;
    }

    public WeatherCoordinationResult coordinate(String routeId,
                                                RouteWeatherAdvisory advisory,
                                                List<PirepIngestResult> pireps) {
        RouteWeatherAdvisory safeAdvisory = advisory == null ? emptyAdvisory() : advisory;
        List<PirepIngestResult> safePireps = pireps == null ? Collections.emptyList() : pireps;
        MeteorologistReviewPriority priority = priorityFor(safeAdvisory, safePireps);
        ZonedDateTime now = ZonedDateTime.now(clock);
        List<WeatherDeskReviewItem> reviews = reviewItems(routeId, safeAdvisory, safePireps, priority, now);
        List<OperationalWeatherConstraint> constraints = constraints(routeId, safeAdvisory);
        List<ControllerHandoffNote> notes = new ArrayList<>();
        notes.add(ControllerHandoffNote.builder()
                .id(routeId + "-weather-handoff")
                .createdAt(now)
                .routeId(routeId)
                .recommendedAction(safeAdvisory.getRecommendedAction())
                .confidence(safeAdvisory.getConfidence())
                .rationale(safeAdvisory.getRationale())
                .build());

        return WeatherCoordinationResult.builder()
                .advisory(WeatherCoordinationAdvisory.builder()
                        .id(routeId + "-weather-advisory")
                        .action(safeAdvisory.getAction())
                        .recommendedAction(safeAdvisory.getRecommendedAction())
                        .severity(safeAdvisory.getSeverity())
                        .reviewPriority(priority)
                        .rationale(safeAdvisory.getRationale())
                        .build())
                .reviewItems(reviews)
                .constraints(constraints)
                .handoffNotes(notes)
                .build();
    }

    private List<WeatherDeskReviewItem> reviewItems(String routeId,
                                                    RouteWeatherAdvisory advisory,
                                                    List<PirepIngestResult> pireps,
                                                    MeteorologistReviewPriority priority,
                                                    ZonedDateTime now) {
        List<WeatherDeskReviewItem> reviews = new ArrayList<>();
        if (advisory.getAction() == WeatherDecisionAction.BLOCKED
                || advisory.getSeverity() == WeatherDecisionSeverity.CRITICAL
                || !advisory.getWarnings().isEmpty()) {
            reviews.add(WeatherDeskReviewItem.builder()
                    .id(routeId + "-route-review")
                    .priority(priority)
                    .createdAt(now)
                    .subject("Route weather decision review")
                    .rationale(advisory.getRationale())
                    .relatedHazardId(advisory.getPrimaryHazardId())
                    .build());
        }
        int index = 0;
        for (PirepIngestResult pirep : pireps) {
            if (pirep == null) {
                continue;
            }
            if (pirep.hasDiagnostic(PirepDiagnosticType.STALE)
                    || pirep.hasDiagnostic(PirepDiagnosticType.MISCODED)
                    || pirep.hasDiagnostic(PirepDiagnosticType.INCOMPLETE)
                    || (pirep.getReport() != null && pirep.getReport().isUrgent())) {
                reviews.add(WeatherDeskReviewItem.builder()
                        .id(routeId + "-pirep-review-" + index++)
                        .priority(pirep.getReport() != null && pirep.getReport().isUrgent()
                                ? MeteorologistReviewPriority.URGENT
                                : priority)
                        .createdAt(now)
                        .subject("PIREP review")
                        .rationale(pirepRationale(pirep))
                        .relatedHazardId(pirep.getReport() == null ? null : pirep.getReport().getId())
                        .build());
            }
        }
        return reviews;
    }

    private List<OperationalWeatherConstraint> constraints(String routeId, RouteWeatherAdvisory advisory) {
        List<OperationalWeatherConstraint> constraints = new ArrayList<>();
        for (RouteBlockagePrediction prediction : advisory.getBlockagePredictions()) {
            if (prediction.isBlocked()) {
                constraints.add(OperationalWeatherConstraint.builder()
                        .id(routeId + "-constraint-" + prediction.getForecastHour())
                        .primaryHazardId(prediction.getPrimaryHazardId())
                        .effectiveStart(prediction.getForecastTime())
                        .effectiveEnd(prediction.getForecastTime())
                        .impactedSegmentIndexes(prediction.getBlockedSegmentIndexes())
                        .recommendedAction(prediction.getRecommendedAction())
                        .rationale(prediction.getRationale())
                        .build());
            }
        }
        return constraints;
    }

    private MeteorologistReviewPriority priorityFor(RouteWeatherAdvisory advisory, List<PirepIngestResult> pireps) {
        if (pireps.stream().anyMatch(p -> p != null && p.getReport() != null && p.getReport().isUrgent())) {
            return MeteorologistReviewPriority.URGENT;
        }
        if (advisory.getSeverity() == WeatherDecisionSeverity.CRITICAL
                || advisory.getAction() == WeatherDecisionAction.BLOCKED) {
            return MeteorologistReviewPriority.URGENT;
        }
        if (advisory.getSeverity() == WeatherDecisionSeverity.WARNING) {
            return MeteorologistReviewPriority.HIGH;
        }
        if (!advisory.getWarnings().isEmpty()) {
            return MeteorologistReviewPriority.NORMAL;
        }
        return MeteorologistReviewPriority.LOW;
    }

    private String pirepRationale(PirepIngestResult result) {
        StringBuilder rationale = new StringBuilder("PIREP requires review");
        for (PirepDiagnostic diagnostic : result.getDiagnostics()) {
            rationale.append("; ").append(diagnostic.getMessage());
        }
        return rationale.toString();
    }

    private RouteWeatherAdvisory emptyAdvisory() {
        return RouteWeatherAdvisory.builder()
                .action(WeatherDecisionAction.CLEAR)
                .recommendedAction(WeatherRecommendedAction.NONE)
                .severity(WeatherDecisionSeverity.INFO)
                .confidence(1.0)
                .rationale("No route weather advisory provided.")
                .build();
    }
}
