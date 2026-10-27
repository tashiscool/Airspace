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
public class AgentDefinition {
    private String agentType;
    private String name;
    private String description;
    private AgenticRiskProfile riskProfile;
    @Builder.Default
    private List<String> recommendedToolIds = new ArrayList<>();
}
