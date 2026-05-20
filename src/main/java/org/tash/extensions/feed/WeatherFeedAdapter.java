package org.tash.extensions.feed;

/**
 * Adapter seam for live or replayed aviation weather products.
 */
public interface WeatherFeedAdapter {
    OperationalFeedPollResult poll();
}
