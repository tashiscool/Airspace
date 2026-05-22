package org.tash.extensions.agentic;

public interface LlmReasoningProvider {
    AgentRunResult reason(AgentRunRequest request, AgentRunResult deterministicDraft);
}
