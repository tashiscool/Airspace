package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;
import org.tash.extensions.engine.CanonicalJson;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.UUID;

@ApplicationScoped
public class AgentAuditService {
    public AgentAuditEnvelope record(AgentRunRequest request, AgentRunResult result, AgentPolicy policy) {
        AgentPolicy safePolicy = policy == null ? AgentPolicy.builder().build() : policy;
        String input = CanonicalJson.write(request == null ? new AgentRunRequest() : request);
        String output = CanonicalJson.write(result == null ? "" : result.toBuilder().auditEnvelope(null).build());
        return AgentAuditEnvelope.builder()
                .id(UUID.nameUUIDFromBytes((input + output).getBytes(java.nio.charset.StandardCharsets.UTF_8)).toString())
                .agentType(result == null ? null : result.getAgentType())
                .agentVersion("deterministic-agentic-1")
                .policyVersion(safePolicy.getVersion())
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .inputHash(CanonicalJson.sha256(input))
                .outputHash(CanonicalJson.sha256(output))
                .citations(result == null ? java.util.Collections.emptyList() : result.getCitations())
                .diagnostics(result == null ? java.util.Collections.singletonList("No result to audit") : result.getDiagnostics())
                .build();
    }
}
