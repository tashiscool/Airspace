package org.tash.extensions.agentic;

import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.AllArgsConstructor;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AgentSourceCitation {
    private String sourceFamily;
    private String sourceId;
    private String label;
    private String route;
    private String ruleId;
    private String sourceSpan;
}
