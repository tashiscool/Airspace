package org.tash.extensions.agentic;

import lombok.Data;

@Data
public class AgentHistoryQuery {
    private Integer limit;
    private String agentType;
    private String missionId;
    private String reservationId;
    private String decisionId;
    private Boolean accepted;
    private String sourceFamily;
    private String taskStatus;
    private String taskPriority;
    private String assignedRole;
    private String routeContains;
}
