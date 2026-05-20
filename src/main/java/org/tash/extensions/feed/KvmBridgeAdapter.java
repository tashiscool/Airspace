package org.tash.extensions.feed;

/**
 * Dependency-light seam for future KVM or waypoint bridge integrations.
 */
public interface KvmBridgeAdapter {
    OperationalFeedPollResult poll();
}
