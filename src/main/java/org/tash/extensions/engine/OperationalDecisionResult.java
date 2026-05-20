package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.carf.api.CarfAnalysisResult;
import org.tash.extensions.messaging.UsnsIngestResult;
import org.tash.extensions.reservation.AirspaceReservation;
import org.tash.extensions.reservation.ReservationConflict;
import org.tash.extensions.weather.coordination.WeatherCoordinationResult;
import org.tash.extensions.weather.decision.RouteBlockagePrediction;
import org.tash.extensions.weather.decision.WeatherDecisionAction;
import org.tash.extensions.weather.decision.WeatherDecisionSeverity;
import org.tash.extensions.weather.decision.WeatherRecommendedAction;
import org.tash.extensions.weather.pirep.PirepIngestResult;
import org.tash.extensions.weather.product.WeatherProduct;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Data
@Builder(toBuilder = true)
public class OperationalDecisionResult {
    private final WeatherDecisionAction action;
    private final WeatherRecommendedAction recommendedAction;
    private final String rationale;
    private final double confidence;
    @Builder.Default private final List<UsnsIngestResult> usnsResults = new ArrayList<>();
    @Builder.Default private final List<CarfAnalysisResult> carfAnalysisResults = new ArrayList<>();
    @Builder.Default private final List<AirspaceReservation> reservations = new ArrayList<>();
    @Builder.Default private final List<ReservationConflict> conflicts = new ArrayList<>();
    @Builder.Default private final List<WeatherProduct> weatherProducts = new ArrayList<>();
    @Builder.Default private final List<PirepIngestResult> pirepResults = new ArrayList<>();
    @Builder.Default private final List<RouteBlockagePrediction> routeBlockages = new ArrayList<>();
    private final WeatherCoordinationResult coordinationResult;
    private final ConstraintFusionResult fusionResult;
    private final DecisionTrace trace;
    private final OperationalDecisionAuditEnvelope auditEnvelope;
    private final OperationalDecisionReplayBundle replayBundle;

    public List<OperationalConstraint> getConstraints() {
        return fusionResult == null ? Collections.emptyList() : fusionResult.getConstraints();
    }

    public String getDecisionSummary() {
        return action + " / " + recommendedAction + " at confidence " + confidence + ": " + rationale;
    }

    public List<OperationalConstraint> getBlockingConstraints() {
        List<OperationalConstraint> blocking = new ArrayList<>();
        for (OperationalConstraint constraint : getConstraints()) {
            if (constraint == null) {
                continue;
            }
            if (constraint.getType() == OperationalConstraintType.ROUTE_BLOCKAGE
                    || constraint.getType() == OperationalConstraintType.CARF_CONFLICT
                    || constraint.getSeverity() == WeatherDecisionSeverity.CRITICAL
                    || constraint.getSeverity() == WeatherDecisionSeverity.WARNING) {
                blocking.add(constraint);
            }
        }
        return Collections.unmodifiableList(blocking);
    }

    public List<String> getWeatherDataWarnings() {
        List<String> warnings = new ArrayList<>();
        if (trace == null) {
            return warnings;
        }
        for (DecisionTraceStep step : trace.getSteps()) {
            if ("warning".equals(step.getStage())) {
                warnings.addAll(step.getWarningIds());
            }
        }
        return Collections.unmodifiableList(warnings);
    }

    public Map<OperationalConstraintType, Long> getConstraintCountsByType() {
        Map<OperationalConstraintType, Long> counts = new LinkedHashMap<>();
        for (OperationalConstraint constraint : getConstraints()) {
            if (constraint == null || constraint.getType() == null) {
                continue;
            }
            counts.put(constraint.getType(), counts.getOrDefault(constraint.getType(), 0L) + 1L);
        }
        return Collections.unmodifiableMap(counts);
    }
}
