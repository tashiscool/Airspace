package org.tash.extensions.feed;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class OperationalFeedPollResult {
    private final boolean accepted;
    @Builder.Default
    private final List<OperationalFeedEnvelope> envelopes = new ArrayList<>();
    @Builder.Default
    private final List<String> diagnostics = new ArrayList<>();

    public List<OperationalFeedEnvelope> getEnvelopes() {
        return Collections.unmodifiableList(envelopes);
    }

    public List<String> getDiagnostics() {
        return Collections.unmodifiableList(diagnostics);
    }
}
