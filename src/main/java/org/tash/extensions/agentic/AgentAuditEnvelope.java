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
public class AgentAuditEnvelope {
    private String id;
    private String agentType;
    private String agentVersion;
    private String policyVersion;
    private ZonedDateTime generatedAt;
    private String inputHash;
    private String outputHash;
    @Builder.Default
    private List<AgentSourceCitation> citations = new ArrayList<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
