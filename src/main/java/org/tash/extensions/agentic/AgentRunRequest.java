package org.tash.extensions.agentic;

import java.util.ArrayList;
import java.util.List;

import lombok.Data;

@Data
public class AgentRunRequest {
    private String agentType;
    private String missionId;
    private String reservationId;
    private String decisionId;
    private String previousDecisionId;
    private String hazardOrDecisionId;
    private String question;
    private String actor;
    private String feedArtifactId;
    private String referenceType;
    private String referenceIdentifier;
    private AgentPolicy policy;
    private List<AgentToolCall> toolCalls = new ArrayList<>();
}
