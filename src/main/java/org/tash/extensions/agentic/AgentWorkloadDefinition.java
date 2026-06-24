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
public class AgentWorkloadDefinition {
    private String id;
    private String label;
    private String category;
    private String gapCoverage;
    private String description;
    private String defaultTrigger;
    private boolean humanApprovalRequired;
    private boolean externalSendAllowed;
    @Builder.Default
    private List<String> policyGuards = new ArrayList<>();
}
