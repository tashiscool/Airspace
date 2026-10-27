package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;

import java.util.ArrayList;
import java.util.List;

@ApplicationScoped
public class LlmReasoningReadinessService {
    public LlmReasoningReadinessResult assess(LlmProviderConfig config, AgentPolicy policy) {
        LlmProviderConfig safe = config == null ? LlmProviderConfig.builder().mode("disabled").build() : config;
        String mode = safe.getMode() == null ? "disabled" : safe.getMode().trim().toLowerCase(java.util.Locale.US);
        List<String> diagnostics = new ArrayList<>();
        if ("disabled".equals(mode) || mode.isEmpty()) {
            diagnostics.add("LLM reasoning disabled; deterministic draft-only output remains authoritative.");
            return LlmReasoningReadinessResult.builder()
                    .ready(false)
                    .modelBacked(false)
                    .networkBacked(false)
                    .diagnostics(diagnostics)
                    .build();
        }
        if ("local-test".equals(mode) || "local-mock".equals(mode)) {
            if (!safe.isLocalMockEnabled()) {
                diagnostics.add("Local mock LLM requested but airspace.agentic.llm.local-mock.enabled=false.");
                return LlmReasoningReadinessResult.builder()
                        .ready(false)
                        .modelBacked(true)
                        .networkBacked(false)
                        .diagnostics(diagnostics)
                        .build();
            }
            diagnostics.add("Local mock LLM readiness accepted; no network call or credential is used.");
            diagnostics.add("Model-backed output remains subject to citation, HITL, policy, and stability gates.");
            return LlmReasoningReadinessResult.builder()
                    .ready(true)
                    .modelBacked(true)
                    .networkBacked(false)
                    .diagnostics(diagnostics)
                    .build();
        }
        if ("http".equals(mode)) {
            if (!safe.isHttpEnabled()) {
                diagnostics.add("HTTP LLM mode requested but airspace.agentic.llm.http.enabled=false.");
                return LlmReasoningReadinessResult.builder()
                        .ready(false)
                        .modelBacked(true)
                        .networkBacked(true)
                        .diagnostics(diagnostics)
                        .build();
            }
            if (safe.getEndpoint() == null || safe.getEndpoint().trim().isEmpty()) {
                diagnostics.add("HTTP LLM mode requested without endpoint.");
                return LlmReasoningReadinessResult.builder()
                        .ready(false)
                        .modelBacked(true)
                        .networkBacked(true)
                        .diagnostics(diagnostics)
                        .build();
            }
            diagnostics.add("HTTP LLM readiness accepted by explicit configuration; output must still pass safety gates.");
            return LlmReasoningReadinessResult.builder()
                    .ready(true)
                    .modelBacked(true)
                    .networkBacked(true)
                    .diagnostics(diagnostics)
                    .build();
        }
        diagnostics.add("Unsupported LLM mode '" + safe.getMode() + "'; deterministic draft retained.");
        return LlmReasoningReadinessResult.builder()
                .ready(false)
                .modelBacked(false)
                .networkBacked(false)
                .diagnostics(diagnostics)
                .build();
    }
}
