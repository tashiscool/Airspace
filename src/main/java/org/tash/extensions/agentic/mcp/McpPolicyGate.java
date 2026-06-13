package org.tash.extensions.agentic.mcp;

import jakarta.enterprise.context.ApplicationScoped;
import org.tash.extensions.agentic.AgentPolicy;

@ApplicationScoped
public class McpPolicyGate {
    public McpInvocationStatus validate(McpServerDefinition server,
                                        McpToolDescriptor tool,
                                        McpToolInvocationRequest request,
                                        AgentPolicy policy) {
        if (server == null || tool == null) {
            return McpInvocationStatus.DENIED;
        }
        AgentPolicy safePolicy = policy == null ? AgentPolicy.builder().build() : policy;
        if (!server.isEnabled() || !tool.isEnabled()) {
            return server.isExternal() || tool.isExternal() ? McpInvocationStatus.SETUP_REQUIRED : McpInvocationStatus.DENIED;
        }
        if ((server.isExternal() || tool.isExternal() || server.isCredentialsRequired() || tool.isCredentialsRequired())
                && (request == null || !request.isExternalConsent())) {
            return McpInvocationStatus.SETUP_REQUIRED;
        }
        if (tool.getSideEffectLevel() == McpSideEffectLevel.MUTATING && !safePolicy.isAllowOfficialStateMutation()) {
            return McpInvocationStatus.DENIED;
        }
        if (tool.getSideEffectLevel() == McpSideEffectLevel.REQUIRES_APPROVAL
                && !safePolicy.isAllowOfficialStateMutation()
                && !safePolicy.isAllowExternalSend()) {
            return McpInvocationStatus.DENIED;
        }
        return McpInvocationStatus.ACCEPTED;
    }

    public String decision(McpInvocationStatus status, McpToolDescriptor tool) {
        if (status == McpInvocationStatus.ACCEPTED) {
            return "ALLOWED_" + (tool == null || tool.getSideEffectLevel() == null ? "READ_ONLY" : tool.getSideEffectLevel().name());
        }
        if (status == McpInvocationStatus.SETUP_REQUIRED) {
            return "SETUP_REQUIRED";
        }
        return "DENIED_BY_AGENT_POLICY";
    }
}
