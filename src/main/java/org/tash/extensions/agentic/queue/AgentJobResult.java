package org.tash.extensions.agentic.queue;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import org.tash.extensions.agentic.AgentRunResult;
import org.tash.extensions.agentic.AgentToolCall;
import org.tash.extensions.agentic.mcp.McpEvidenceReceipt;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AgentJobResult {
    private String id;
    private AgentJobStatus status;
    private AgentJobRequest request;
    private AgentRunResult runResult;
    private ZonedDateTime createdAt;
    private ZonedDateTime startedAt;
    private ZonedDateTime completedAt;
    @Builder.Default
    private List<AgentToolCall> toolCalls = new ArrayList<>();
    @Builder.Default
    private List<McpEvidenceReceipt> receipts = new ArrayList<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
