package org.tash.extensions.agentic;

import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.AllArgsConstructor;

import java.time.ZonedDateTime;
import java.util.LinkedHashMap;
import java.util.Map;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AgentToolCall {
    private String id;
    private String toolName;
    private String status;
    private ZonedDateTime startedAt;
    private ZonedDateTime completedAt;
    @Builder.Default
    private Map<String, Object> arguments = new LinkedHashMap<>();
    private String resultSummary;
}
