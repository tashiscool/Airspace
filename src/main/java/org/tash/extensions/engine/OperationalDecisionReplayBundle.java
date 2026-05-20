package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

@Data
@Builder(toBuilder = true)
public class OperationalDecisionReplayBundle {
    private final String bundleVersion;
    private final OperationalDecisionRequest request;
    private final String ruleCatalogVersion;
    private final String engineVersion;
    private final String expectedAction;
    private final String expectedRecommendedAction;
    private final String expectedResultHash;
    private final OperationalDecisionAuditEnvelope auditEnvelope;
    @Builder.Default
    private final List<Map<String, Object>> decodedProductSummaries = new ArrayList<>();

    public List<Map<String, Object>> getDecodedProductSummaries() {
        return Collections.unmodifiableList(decodedProductSummaries == null ? Collections.emptyList() : decodedProductSummaries);
    }
}
