package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;

import java.util.ArrayList;
import java.util.List;

@ApplicationScoped
public class LocalReplayValidationHarness implements LiveFeedValidationHarness {
    @Override
    public ValidationResult validateLocalReplay(List<String> payloads) {
        List<String> diagnostics = new ArrayList<>();
        if (payloads == null || payloads.isEmpty()) {
            diagnostics.add("No local replay payloads supplied.");
        } else {
            for (int index = 0; index < payloads.size(); index++) {
                if (payloads.get(index) == null || payloads.get(index).trim().isEmpty()) {
                    diagnostics.add("Payload " + index + " is blank.");
                }
            }
        }
        return ValidationResult.builder()
                .accepted(diagnostics.isEmpty())
                .payloadCount(payloads == null ? 0 : payloads.size())
                .diagnostics(diagnostics)
                .build();
    }
}
