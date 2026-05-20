package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class DecisionRuleRef {
    private final String id;
    private final String catalogVersion;
    private final String category;
    private final String severity;

    public static DecisionRuleRef of(DecisionRule rule) {
        return rule == null ? null : DecisionRuleRef.builder()
                .id(rule.getId())
                .catalogVersion(DecisionRuleCatalog.version())
                .category(rule.getCategory())
                .severity(rule.getSeverity())
                .build();
    }
}
