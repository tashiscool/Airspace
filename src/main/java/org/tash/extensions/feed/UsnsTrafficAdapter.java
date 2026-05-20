package org.tash.extensions.feed;

/**
 * Adapter seam for live or replayed USNS/NADIN/WMSCR traffic.
 */
public interface UsnsTrafficAdapter {
    OperationalFeedPollResult poll();
}
