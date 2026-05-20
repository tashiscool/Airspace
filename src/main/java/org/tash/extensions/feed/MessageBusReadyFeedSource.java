package org.tash.extensions.feed;

/**
 * Broker-shaped feed seam for Kafka, Rabbit, SWIM, or KVM-style adapters.
 *
 * Implementations can bind this to a live broker later; the engine only needs
 * the polling contract and stable source metadata.
 */
public interface MessageBusReadyFeedSource extends OperationalFeedSource {
    String transportName();

    String topicOrQueue();
}
