package org.tash.extensions.feed;

import java.util.List;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.UUID;

/**
 * Local replay adapter that presents supplied text as weather traffic.
 */
public class LocalWeatherFeedAdapter implements WeatherFeedAdapter {
    private final InMemoryOperationalFeedSource source;

    public LocalWeatherFeedAdapter(String sourceId, List<String> products) {
        this.source = new InMemoryOperationalFeedSource(sourceId, envelopes(sourceId, products));
    }

    @Override
    public OperationalFeedPollResult poll() {
        return source.poll();
    }

    private List<OperationalFeedEnvelope> envelopes(String sourceId, List<String> products) {
        List<OperationalFeedEnvelope> values = new ArrayList<>();
        if (products != null) {
            for (String product : products) {
                values.add(OperationalFeedEnvelope.builder()
                        .id(UUID.randomUUID().toString())
                        .sourceId(sourceId == null ? "local-weather" : sourceId)
                        .type(OperationalFeedType.WEATHER)
                        .receivedAt(ZonedDateTime.now(ZoneOffset.UTC))
                        .rawPayload(product)
                        .build());
            }
        }
        return values;
    }
}
