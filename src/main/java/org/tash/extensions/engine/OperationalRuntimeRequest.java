package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.feed.OperationalFeedSource;
import org.tash.extensions.ops.OperationalMetricSink;
import org.tash.extensions.repository.AuditReplayRepository;
import org.tash.extensions.repository.OperationalDecisionRepository;
import org.tash.extensions.repository.PirepReportRepository;
import org.tash.extensions.repository.WeatherProductRepository;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class OperationalRuntimeRequest {
    @Builder.Default
    private final List<OperationalFeedSource> feedSources = new ArrayList<>();
    private final OperationalDecisionRequest decisionRequest;
    private final OperationalDecisionRepository decisionRepository;
    private final AuditReplayRepository auditReplayRepository;
    private final WeatherProductRepository weatherProductRepository;
    private final PirepReportRepository pirepReportRepository;
    private final OperationalMetricSink metricSink;
    @Builder.Default
    private final boolean persistArtifacts = true;

    public List<OperationalFeedSource> getFeedSources() {
        return Collections.unmodifiableList(feedSources == null ? Collections.emptyList() : feedSources);
    }
}
