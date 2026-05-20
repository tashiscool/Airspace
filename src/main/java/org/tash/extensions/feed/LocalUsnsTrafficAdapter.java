package org.tash.extensions.feed;

import java.util.List;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.UUID;

/**
 * Local replay adapter that presents supplied text as USNS traffic.
 */
public class LocalUsnsTrafficAdapter implements UsnsTrafficAdapter {
    private final InMemoryOperationalFeedSource source;

    public LocalUsnsTrafficAdapter(String sourceId, List<String> messages) {
        this.source = new InMemoryOperationalFeedSource(sourceId, envelopes(sourceId, messages));
    }

    @Override
    public OperationalFeedPollResult poll() {
        return source.poll();
    }

    private List<OperationalFeedEnvelope> envelopes(String sourceId, List<String> messages) {
        List<OperationalFeedEnvelope> values = new ArrayList<>();
        if (messages != null) {
            for (String message : messages) {
                values.add(OperationalFeedEnvelope.builder()
                        .id(UUID.randomUUID().toString())
                        .sourceId(sourceId == null ? "local-usns" : sourceId)
                        .type(OperationalFeedType.USNS)
                        .receivedAt(ZonedDateTime.now(ZoneOffset.UTC))
                        .rawPayload(message)
                        .build());
            }
        }
        return values;
    }
}
