package org.tash.extensions.feed;

/**
 * Adapter seam for live or replayed aviation weather products.
 */
public interface WeatherFeedAdapter {
    OperationalFeedPollResult poll();

    default WeatherFeedBatch poll(WeatherFeedPollRequest request) {
        OperationalFeedPollResult result = poll();
        return WeatherFeedBatch.builder()
                .accepted(result != null && result.isAccepted())
                .sourceId(result == null || result.getEnvelopes().isEmpty() ? null : result.getEnvelopes().get(0).getSourceId())
                .pollResult(result)
                .diagnostics(result == null ? java.util.Collections.singletonList("Weather adapter returned no poll result") : result.getDiagnostics())
                .receivedAt(java.time.ZonedDateTime.now(java.time.ZoneOffset.UTC))
                .build();
    }
}
