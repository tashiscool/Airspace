package org.tash.extensions.feed;

public interface OperationalFeedSource {
    String sourceId();

    OperationalFeedPollResult poll();
}
