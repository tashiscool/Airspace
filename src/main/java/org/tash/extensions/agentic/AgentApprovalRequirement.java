package org.tash.extensions.agentic;

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
public class AgentApprovalRequirement {
    private String id;
    private HumanReviewMode mode;
    private String reason;
    private String route;
    private boolean required;
    @Builder.Default
    private List<AgentSourceCitation> citations = new ArrayList<>();
}
