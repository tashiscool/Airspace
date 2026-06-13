package org.tash.extensions.agentic.stability;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AgentStabilityResult {
    private String id;
    private boolean accepted;
    private int iterations;
    private ZonedDateTime startedAt;
    private ZonedDateTime completedAt;
    @Builder.Default
    private List<String> runIds = new ArrayList<>();
    @Builder.Default
    private List<AgentStabilityMetric> metrics = new ArrayList<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
