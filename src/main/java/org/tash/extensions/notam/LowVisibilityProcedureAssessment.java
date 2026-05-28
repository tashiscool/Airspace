package org.tash.extensions.notam;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class LowVisibilityProcedureAssessment {
    private final String airportId;
    private final Double reportedRvrFeet;
    private final String rvrEquipmentStatus;
    private final String lowVisibilityProcedureTerminology;
    private final String recommendedAction;
    private final String actionSublabel;
    private final boolean ambiguityDetected;
    private final boolean separateArtifactsRetained;
    @Builder.Default
    private final List<String> sourceRefs = new ArrayList<>();
    @Builder.Default
    private final List<String> reviewMessages = new ArrayList<>();
    @Builder.Default
    private final List<String> diagnostics = new ArrayList<>();

    public List<String> getSourceRefs() {
        return Collections.unmodifiableList(sourceRefs == null ? Collections.emptyList() : sourceRefs);
    }

    public List<String> getReviewMessages() {
        return Collections.unmodifiableList(reviewMessages == null ? Collections.emptyList() : reviewMessages);
    }

    public List<String> getDiagnostics() {
        return Collections.unmodifiableList(diagnostics == null ? Collections.emptyList() : diagnostics);
    }
}
