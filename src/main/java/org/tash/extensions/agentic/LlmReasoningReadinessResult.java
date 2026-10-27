package org.tash.extensions.agentic;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.util.ArrayList;
import java.util.List;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class LlmReasoningReadinessResult {
    private boolean ready;
    private boolean modelBacked;
    private boolean networkBacked;
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
