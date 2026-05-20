package org.tash.extensions.weather.decision;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.engine.DecisionRuleApplication;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class RouteImpactScoringBreakdown {
    private final double severityWeight;
    private final double growthAdjustment;
    private final double stormPhaseWeight;
    private final double echoTopFactor;
    private final double leadTimeConfidence;
    private final double calibratedScore;
    private final double blockedProbability;
    private final double deviationLikelihood;
    private final double capacityImpact;
    @Builder.Default
    private final List<DecisionRuleApplication> ruleApplications = new ArrayList<>();

    public List<DecisionRuleApplication> getRuleApplications() {
        return Collections.unmodifiableList(ruleApplications == null ? Collections.emptyList() : ruleApplications);
    }
}
