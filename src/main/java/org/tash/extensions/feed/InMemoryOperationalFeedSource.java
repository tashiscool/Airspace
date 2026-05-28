package org.tash.extensions.feed;

import java.util.ArrayList;
import java.util.List;

public class InMemoryOperationalFeedSource implements OperationalFeedSource {
    private final String sourceId;
    private final List<OperationalFeedEnvelope> envelopes = new ArrayList<>();

    public InMemoryOperationalFeedSource(String sourceId, List<OperationalFeedEnvelope> envelopes) {
        this.sourceId = sourceId == null ? "memory-feed" : sourceId;
        if (envelopes != null) {
            this.envelopes.addAll(envelopes);
        }
    }

    @Override
    public String sourceId() {
        return sourceId;
    }

    @Override
    public OperationalFeedPollResult poll() {
        return OperationalFeedPollResult.builder()
                .accepted(true)
                .envelopes(new ArrayList<>(envelopes))
                .typedDiagnostics(List.of(OperationalFeedDiagnostic.builder()
                        .authorityMode(OperationalFeedAuthorityMode.LOCAL_FIXTURE)
                        .code("LOCAL_REPLAY")
                        .sourceId(sourceId)
                        .message("Local replay/fixture feed source; not a confirmed live authoritative feed.")
                        .build()))
                .build();
    }
}
