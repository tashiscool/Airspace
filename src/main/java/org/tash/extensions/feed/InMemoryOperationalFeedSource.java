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
                .build();
    }
}
