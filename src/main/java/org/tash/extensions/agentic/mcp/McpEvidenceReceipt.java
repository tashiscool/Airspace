package org.tash.extensions.agentic.mcp;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import org.tash.extensions.agentic.AgentSourceCitation;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class McpEvidenceReceipt {
    private String id;
    private String serverId;
    private String toolId;
    private McpSideEffectLevel sideEffectLevel;
    private McpInvocationStatus status;
    private String policyDecision;
    private String redactionStatus;
    private String inputHash;
    private String outputHash;
    private String inputSummary;
    private String outputSummary;
    private ZonedDateTime startedAt;
    private ZonedDateTime completedAt;
    private long durationMillis;
    @Builder.Default
    private List<AgentSourceCitation> sourceRefs = new ArrayList<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
