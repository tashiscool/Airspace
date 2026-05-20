package org.tash.extensions.uncertainty;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class UncertaintyAssessmentResult {
    private final double adjustedConfidence;
    private final double confidencePenalty;
    private final double expandedCorridorNauticalMiles;
    private final String rationale;
    @Builder.Default
    private final List<String> warnings = new ArrayList<>();

    public List<String> getWarnings() {
        return Collections.unmodifiableList(warnings == null ? Collections.emptyList() : warnings);
    }
}
