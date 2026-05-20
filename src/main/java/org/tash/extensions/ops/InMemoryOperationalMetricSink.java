package org.tash.extensions.ops;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class InMemoryOperationalMetricSink implements OperationalMetricSink {
    private final List<OperationalMetric> metrics = new ArrayList<>();

    @Override
    public synchronized void record(OperationalMetric metric) {
        if (metric != null) {
            metrics.add(metric);
        }
    }

    public synchronized List<OperationalMetric> metrics() {
        return Collections.unmodifiableList(new ArrayList<>(metrics));
    }

    public synchronized double sum(String name) {
        return metrics.stream()
                .filter(metric -> name.equals(metric.getName()))
                .mapToDouble(OperationalMetric::getValue)
                .sum();
    }

    public synchronized Map<String, Double> summary() {
        Map<String, Double> out = new LinkedHashMap<>();
        for (OperationalMetric metric : metrics) {
            out.put(metric.getName(), out.getOrDefault(metric.getName(), 0.0) + metric.getValue());
        }
        return Collections.unmodifiableMap(out);
    }
}
