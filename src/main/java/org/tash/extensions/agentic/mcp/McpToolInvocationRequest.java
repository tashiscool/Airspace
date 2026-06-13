package org.tash.extensions.agentic.mcp;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import org.tash.extensions.agentic.AgentPolicy;

import java.util.LinkedHashMap;
import java.util.Map;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class McpToolInvocationRequest {
    private String serverId;
    private String toolId;
    private String actor;
    private boolean externalConsent;
    private AgentPolicy policy;
    @Builder.Default
    private Map<String, Object> arguments = new LinkedHashMap<>();
}
