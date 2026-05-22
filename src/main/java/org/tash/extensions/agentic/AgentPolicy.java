package org.tash.extensions.agentic;

import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.AllArgsConstructor;

import java.util.Arrays;
import java.util.List;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AgentPolicy {
    @Builder.Default
    private String version = "agent-policy-2026-05-22";
    @Builder.Default
    private boolean allowOfficialStateMutation = false;
    @Builder.Default
    private boolean allowExternalSend = false;
    @Builder.Default
    private boolean requireHumanApprovalForDrafts = true;
    @Builder.Default
    private boolean requireCitationsForOperationalClaims = true;
    @Builder.Default
    private List<String> allowedActions = Arrays.asList("ANALYZE", "DRAFT", "TRIAGE", "BRIEF", "EXPLAIN");
}
