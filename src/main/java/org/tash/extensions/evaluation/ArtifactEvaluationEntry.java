package org.tash.extensions.evaluation;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class ArtifactEvaluationEntry {
    private String path;
    private ArtifactEvaluationStatus status;
    private String category;
    private String finding;
    private String testReference;
}
