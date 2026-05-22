package org.tash.extensions.agentic;

import lombok.Data;

@Data
public class AgentTaskTransitionRequest {
    private String status;
    private String actor;
    private String note;
}
