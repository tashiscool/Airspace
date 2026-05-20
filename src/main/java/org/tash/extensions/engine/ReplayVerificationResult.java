package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class ReplayVerificationResult {
    private final boolean accepted;
    private final OperationalDecisionResult result;
    @Builder.Default
    private final List<String> warnings = new ArrayList<>();
    @Builder.Default
    private final List<String> errors = new ArrayList<>();

    public List<String> getWarnings() {
        return Collections.unmodifiableList(warnings == null ? Collections.emptyList() : warnings);
    }

    public List<String> getErrors() {
        return Collections.unmodifiableList(errors == null ? Collections.emptyList() : errors);
    }
}
