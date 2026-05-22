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
public class AgentFinding {
    private String id;
    private String category;
    private String severity;
    private String message;
    private double confidence;
    @Builder.Default
    private List<AgentSourceCitation> citations = new ArrayList<>();
}
