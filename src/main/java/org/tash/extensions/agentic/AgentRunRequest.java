package org.tash.extensions.agentic;

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
    private AgentPolicy policy;
}
