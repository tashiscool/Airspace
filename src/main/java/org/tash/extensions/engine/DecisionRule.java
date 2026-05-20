package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class DecisionRule {
    private final String id;
    private final String description;
    private final String category;
    private final String severity;
    private final String defaultRationaleTemplate;
    @Builder.Default
    private final List<DecisionThreshold> defaultThresholds = new ArrayList<>();

    public List<DecisionThreshold> getDefaultThresholds() {
        return Collections.unmodifiableList(defaultThresholds == null ? Collections.emptyList() : defaultThresholds);
    }
}
