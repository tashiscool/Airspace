package org.tash.extensions.agentic;

import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.AllArgsConstructor;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AgentOperationalDelta {
    private String id;
    private String changeType;
    private String sourceFamily;
    private String sourceId;
    private String previousValue;
    private String currentValue;
    private String severity;
    private ZonedDateTime observedAt;
    @Builder.Default
    private List<AgentSourceCitation> citations = new ArrayList<>();
}
