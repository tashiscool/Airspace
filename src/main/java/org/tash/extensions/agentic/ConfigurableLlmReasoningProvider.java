package org.tash.extensions.agentic;

import com.fasterxml.jackson.databind.ObjectMapper;
import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;
import org.eclipse.microprofile.config.inject.ConfigProperty;
import org.tash.extensions.engine.CanonicalJson;

import java.io.IOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

@ApplicationScoped
public class ConfigurableLlmReasoningProvider implements LlmReasoningProvider {
    @ConfigProperty(name = "airspace.agentic.llm.mode", defaultValue = "disabled")
    String mode;
    @ConfigProperty(name = "airspace.agentic.llm.endpoint")
    Optional<String> endpoint;
    @ConfigProperty(name = "airspace.agentic.llm.api-key")
    Optional<String> apiKey;
    @ConfigProperty(name = "airspace.agentic.llm.model", defaultValue = "not-configured")
    String model;
    @ConfigProperty(name = "airspace.agentic.llm.timeout-ms", defaultValue = "2500")
    int timeoutMillis;
    @Inject
    ObjectMapper mapper;

    @Override
    public AgentRunResult reason(AgentRunRequest request, AgentRunResult deterministicDraft) {
        LlmProviderConfig config = config();
        if (!config.enabled()) {
            return deterministicDraft;
        }
        if ("local-test".equalsIgnoreCase(config.getMode())) {
            return localTestReasoning(deterministicDraft, config);
        }
        if ("http".equalsIgnoreCase(config.getMode())) {
            return httpReasoning(request, deterministicDraft, config);
        }
        return deterministicDraft.toBuilder()
                .diagnostics(withDiagnostic(deterministicDraft, "Unsupported LLM mode '" + config.getMode() + "'; deterministic draft retained."))
                .build();
    }

    private AgentRunResult localTestReasoning(AgentRunResult draft, LlmProviderConfig config) {
        List<AgentFinding> findings = new ArrayList<>(draft.getFindings());
        List<AgentSourceCitation> citations = draft.getCitations().isEmpty()
                ? java.util.Collections.singletonList(AgentSupport.citation("ENGINE", "agentic", "Deterministic agent draft", "/decisions/latest"))
                : draft.getCitations();
        findings.add(AgentFinding.builder()
                .id(AgentSupport.id("finding", "llm-local-test:" + draft.getId()))
                .category("LLM_REASONING")
                .severity("INFO")
                .message("Configured local LLM test provider reviewed the deterministic draft and retained engine authority.")
                .confidence(0.7)
                .citations(citations)
                .build());
        return draft.toBuilder()
                .findings(findings)
                .citations(citations)
                .reasoningEnvelope(draft.getReasoningEnvelope() == null ? null : draft.getReasoningEnvelope().toBuilder()
                        .modelId(config.getModel())
                        .reasoningMode("LOCAL_TEST_PROVIDER")
                        .build())
                .diagnostics(withDiagnostic(draft, "Local test LLM provider enabled; no external model call was made."))
                .build();
    }

    private AgentRunResult httpReasoning(AgentRunRequest request, AgentRunResult draft, LlmProviderConfig config) {
        if (config.getEndpoint() == null || config.getEndpoint().trim().isEmpty()) {
            return draft.toBuilder()
                    .diagnostics(withDiagnostic(draft, "HTTP LLM mode requested without endpoint; deterministic draft retained."))
                    .build();
        }
        try {
            String body = mapper.writeValueAsString(new LlmHttpPayload(request, draft.getReasoningEnvelope(), draft));
            HttpRequest.Builder builder = HttpRequest.newBuilder(URI.create(config.getEndpoint()))
                    .timeout(Duration.ofMillis(Math.max(250, config.getTimeoutMillis())))
                    .header("Content-Type", "application/json")
                    .POST(HttpRequest.BodyPublishers.ofString(body));
            if (config.getApiKey() != null && !config.getApiKey().trim().isEmpty()) {
                builder.header("Authorization", "Bearer " + config.getApiKey());
            }
            HttpResponse<String> response = HttpClient.newHttpClient().send(builder.build(), HttpResponse.BodyHandlers.ofString());
            if (response.statusCode() < 200 || response.statusCode() >= 300) {
                return draft.toBuilder()
                        .diagnostics(withDiagnostic(draft, "HTTP LLM provider returned " + response.statusCode() + "; deterministic draft retained."))
                        .build();
            }
            AgentRunResult reasoned = mapper.readValue(response.body(), AgentRunResult.class);
            return reasoned.toBuilder()
                    .reasoningEnvelope(reasoned.getReasoningEnvelope() == null ? draft.getReasoningEnvelope() : reasoned.getReasoningEnvelope())
                    .diagnostics(withDiagnostic(reasoned, "HTTP LLM provider response merged; safety validation still required."))
                    .build();
        } catch (IOException | InterruptedException | IllegalArgumentException ex) {
            if (ex instanceof InterruptedException) {
                Thread.currentThread().interrupt();
            }
            return draft.toBuilder()
                    .diagnostics(withDiagnostic(draft, "HTTP LLM provider failed: " + ex.getClass().getSimpleName() + "; deterministic draft retained."))
                    .build();
        }
    }

    private LlmProviderConfig config() {
        return LlmProviderConfig.builder()
                .mode(mode)
                .endpoint(endpoint.orElse(null))
                .apiKey(apiKey.orElse(null))
                .model(model)
                .timeoutMillis(timeoutMillis)
                .build();
    }

    private List<String> withDiagnostic(AgentRunResult result, String diagnostic) {
        List<String> diagnostics = new ArrayList<>(result.getDiagnostics());
        diagnostics.add(diagnostic);
        return diagnostics;
    }

    private static class LlmHttpPayload {
        public final AgentRunRequest request;
        public final AgentReasoningEnvelope reasoningEnvelope;
        public final AgentRunResult deterministicDraft;
        public final String deterministicDraftHash;

        LlmHttpPayload(AgentRunRequest request, AgentReasoningEnvelope reasoningEnvelope, AgentRunResult deterministicDraft) {
            this.request = request;
            this.reasoningEnvelope = reasoningEnvelope;
            this.deterministicDraft = deterministicDraft;
            this.deterministicDraftHash = CanonicalJson.sha256(CanonicalJson.write(deterministicDraft));
        }
    }
}
