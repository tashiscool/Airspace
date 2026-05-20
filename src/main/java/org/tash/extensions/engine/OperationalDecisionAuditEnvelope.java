package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

@Data
@Builder
public class OperationalDecisionAuditEnvelope {
    private final String engineVersion;
    private final String ruleCatalogVersion;
    private final String requestHash;
    private final String resultHash;
    private final String configHash;
    private final ZonedDateTime timestamp;
    private final String signingKeyId;
    private final String signature;
    @Builder.Default
    private final List<String> inputSourceHashes = new ArrayList<>();
    @Builder.Default
    private final Map<String, Object> configSnapshot = Collections.emptyMap();

    public List<String> getInputSourceHashes() {
        return Collections.unmodifiableList(inputSourceHashes == null ? Collections.emptyList() : inputSourceHashes);
    }
}
