package org.tash.extensions.agentic;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.time.ZonedDateTime;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AgentStoreStatus {
    private String mode;
    private boolean durable;
    private String path;
    private int runCount;
    private int taskCount;
    private ZonedDateTime latestRunAt;
}
