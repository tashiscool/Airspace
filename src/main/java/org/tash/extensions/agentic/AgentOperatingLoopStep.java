package org.tash.extensions.agentic;

import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.AllArgsConstructor;

import java.util.ArrayList;
import java.util.List;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AgentOperatingLoopStep {
    private String stage;
    private String status;
    private String summary;
    @Builder.Default
    private List<AgentSourceCitation> citations = new ArrayList<>();
}
