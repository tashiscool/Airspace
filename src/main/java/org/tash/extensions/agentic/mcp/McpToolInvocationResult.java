package org.tash.extensions.agentic.mcp;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import org.tash.extensions.agentic.AgentSourceCitation;
import org.tash.extensions.agentic.AgentToolCall;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class McpToolInvocationResult {
    private String id;
    private String serverId;
    private String toolId;
    private McpInvocationStatus status;
    private String policyDecision;
    private ZonedDateTime startedAt;
    private ZonedDateTime completedAt;
    private Object result;
    private String resultSummary;
    private McpEvidenceReceipt evidenceReceipt;
    private AgentToolCall toolCall;
    @Builder.Default
    private List<AgentSourceCitation> sourceRefs = new ArrayList<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
