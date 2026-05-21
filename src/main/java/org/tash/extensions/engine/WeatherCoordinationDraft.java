package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class WeatherCoordinationDraft {
    private final String family;
    private final String subject;
    private final String body;
    @Builder.Default private final List<String> recipients = new ArrayList<>();
    @Builder.Default private final List<DecisionSourceRef> sourceRefs = new ArrayList<>();

    public List<String> getRecipients() {
        return Collections.unmodifiableList(recipients == null ? Collections.emptyList() : recipients);
    }

    public List<DecisionSourceRef> getSourceRefs() {
        return Collections.unmodifiableList(sourceRefs == null ? Collections.emptyList() : sourceRefs);
    }
}
