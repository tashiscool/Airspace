package org.tash.extensions.agentic;

import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.AllArgsConstructor;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class LlmProviderConfig {
    private String mode;
    private String endpoint;
    private String apiKey;
    private String model;
    private int timeoutMillis;

    public boolean enabled() {
        return mode != null && !"disabled".equalsIgnoreCase(mode.trim());
    }
}
