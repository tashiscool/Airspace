package org.tash.extensions.ops;

public class NoopOperationalMetricSink implements OperationalMetricSink {
    @Override
    public void record(OperationalMetric metric) {
        // Intentionally empty.
    }
}
