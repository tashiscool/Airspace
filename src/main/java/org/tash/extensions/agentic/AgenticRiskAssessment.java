package org.tash.extensions.agentic;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.util.ArrayList;
import java.util.List;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AgenticRiskAssessment {
    private String id;
    private String subjectType;
    private String subjectId;
    private String summary;
    private AgenticRiskProfile riskProfile;
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
