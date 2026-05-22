package org.tash.extensions.agentic;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.List;

public interface LiveFeedValidationHarness {
    ValidationResult validateLocalReplay(List<String> payloads);

    @Data
    @Builder
    class ValidationResult {
        private boolean accepted;
        private int payloadCount;
        @Builder.Default
        private List<String> diagnostics = new ArrayList<>();
    }
}
