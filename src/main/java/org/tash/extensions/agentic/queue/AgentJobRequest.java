package org.tash.extensions.agentic.queue;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import org.tash.extensions.agentic.AgentRunRequest;

import java.util.LinkedHashMap;
import java.util.Map;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AgentJobRequest {
    private String idempotencyKey;
    private String actor;
    private AgentRunRequest agentRunRequest;
    @Builder.Default
    private Map<String, Map<String, Object>> toolArguments = new LinkedHashMap<>();
}
