package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class DecisionRuleApplication {
    private final DecisionRuleRef rule;
    private final String input;
    private final String formula;
    private final Double observedValue;
    private final Double outputValue;
    private final String rationale;
    @Builder.Default
    private final List<DecisionThreshold> thresholds = new ArrayList<>();

    public List<DecisionThreshold> getThresholds() {
        return Collections.unmodifiableList(thresholds == null ? Collections.emptyList() : thresholds);
    }
}
