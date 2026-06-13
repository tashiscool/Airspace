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
public class AgentTask {
    private String id;
    private String title;
    private String status;
    private String priority;
    private String assignedRole;
    private String route;
    private String rationale;
    private HumanReviewMode humanReviewMode;
    private String humanReviewReason;
    @Builder.Default
    private List<AgentSourceCitation> citations = new ArrayList<>();
}
