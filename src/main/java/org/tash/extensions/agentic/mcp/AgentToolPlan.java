package org.tash.extensions.agentic.mcp;

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
public class AgentToolPlan {
    @Builder.Default
    private List<McpToolDescriptor> availableTools = new ArrayList<>();
    @Builder.Default
    private List<McpToolDescriptor> blockedTools = new ArrayList<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
