package org.tash.extensions.engine;

import org.tash.extensions.feed.OperationalFeedBatchResult;
import org.tash.extensions.feed.OperationalFeedEnvelope;
import org.tash.extensions.feed.OperationalFeedIngestResult;
import org.tash.extensions.feed.OperationalFeedIngestService;
import org.tash.extensions.feed.OperationalFeedPollResult;
import org.tash.extensions.ops.DecisionEngineMetrics;
import org.tash.extensions.ops.FeedIngestMetrics;
import org.tash.extensions.ops.InMemoryOperationalMetricSink;
import org.tash.extensions.ops.OperationalMetricSink;
import org.tash.extensions.repository.AuditReplayRepository;
import org.tash.extensions.repository.OperationalDecisionRepository;
import org.tash.extensions.repository.PirepReportRepository;
import org.tash.extensions.repository.WeatherProductRepository;
import org.tash.extensions.weather.product.WeatherProduct;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class OperationalRuntimeService {
    private final OperationalDecisionEngine decisionEngine;
    private final OperationalFeedIngestService feedIngestService;

    public OperationalRuntimeService() {
        this(new OperationalDecisionEngine(), new OperationalFeedIngestService());
    }

    public OperationalRuntimeService(OperationalDecisionEngine decisionEngine,
                                     OperationalFeedIngestService feedIngestService) {
        this.decisionEngine = decisionEngine == null ? new OperationalDecisionEngine() : decisionEngine;
        this.feedIngestService = feedIngestService == null ? new OperationalFeedIngestService() : feedIngestService;
    }

    public OperationalRuntimeResult run(OperationalRuntimeRequest request) {
        OperationalRuntimeRequest safe = request == null ? OperationalRuntimeRequest.builder().build() : request;
        OperationalMetricSink sink = safe.getMetricSink();
        List<OperationalFeedBatchResult> batches = new ArrayList<>();
        List<OperationalFeedEnvelope> feedArtifacts = new ArrayList<>();
        OperationalDecisionRequest baseRequest = baseDecisionRequest(safe.getDecisionRequest());
        OperationalDecisionRequest.OperationalDecisionRequestBuilder decisionBuilder = baseRequest.toBuilder();
        List<String> rawUsns = new ArrayList<>(baseRequest.getRawUsnsMessages());
        List<String> rawCarf = new ArrayList<>(baseRequest.getRawCarfMessages());
        List<WeatherProduct> weatherProducts = new ArrayList<>(baseRequest.getWeatherProducts());
        List<org.tash.extensions.weather.pirep.PirepReport> pireps = new ArrayList<>(baseRequest.getPireps());
        List<org.tash.extensions.notam.NotamAirspaceRestriction> notams = new ArrayList<>(baseRequest.getNotamRestrictions());

        Instant ingestStart = Instant.now();
        for (org.tash.extensions.feed.OperationalFeedSource source : safe.getFeedSources()) {
            OperationalFeedPollResult poll = source.poll();
            OperationalFeedBatchResult batch = feedIngestService.ingest(poll);
            batches.add(batch);
            FeedIngestMetrics.recordBatch(sink, batch);
            mergeFeedArtifacts(batch, feedArtifacts, rawUsns, rawCarf, weatherProducts, pireps, notams);
        }
        DecisionEngineMetrics.recordDuration(sink, "feed-ingest",
                Duration.between(ingestStart, Instant.now()).toMillis());

        OperationalDecisionRequest decisionRequest = decisionBuilder
                .rawUsnsMessages(rawUsns)
                .rawCarfMessages(rawCarf)
                .weatherProducts(weatherProducts)
                .pireps(pireps)
                .notamRestrictions(notams)
                .feedArtifacts(feedArtifacts)
                .build();
        Instant decisionStart = Instant.now();
        OperationalDecisionResult decisionResult = decisionEngine.evaluate(decisionRequest);
        DecisionEngineMetrics.recordDuration(sink, "decision-evaluate",
                Duration.between(decisionStart, Instant.now()).toMillis());
        DecisionEngineMetrics.recordDecision(sink, decisionResult);

        Map<String, String> persisted = safe.isPersistArtifacts()
                ? persist(safe, decisionResult)
                : new LinkedHashMap<>();
        Map<String, Double> metrics = sink instanceof InMemoryOperationalMetricSink
                ? ((InMemoryOperationalMetricSink) sink).summary()
                : new LinkedHashMap<>();
        decisionResult = decisionResult.toBuilder()
                .persistedArtifactIds(persisted)
                .metricsSummary(metrics)
                .build();
        return OperationalRuntimeResult.builder()
                .feedBatchResults(batches)
                .decisionResult(decisionResult)
                .persistedArtifactIds(persisted)
                .metricsSummary(metrics)
                .accepted(decisionResult.getAction() != null)
                .build();
    }

    private OperationalDecisionRequest baseDecisionRequest(OperationalDecisionRequest request) {
        return request == null ? OperationalDecisionRequest.builder().build() : request;
    }

    private void mergeFeedArtifacts(OperationalFeedBatchResult batch,
                                    List<OperationalFeedEnvelope> feedArtifacts,
                                    List<String> rawUsns,
                                    List<String> rawCarf,
                                    List<WeatherProduct> weatherProducts,
                                    List<org.tash.extensions.weather.pirep.PirepReport> pireps,
                                    List<org.tash.extensions.notam.NotamAirspaceRestriction> notams) {
        for (OperationalFeedIngestResult result : batch.getResults()) {
            if (result.getEnvelope() != null) {
                feedArtifacts.add(result.getEnvelope());
            }
            if (result.getEnvelope() != null && result.getUsnsResult() != null) {
                rawUsns.add(result.getEnvelope().getRawPayload());
            }
            if (result.getCarfResult() != null && result.getEnvelope() != null) {
                rawCarf.add(result.getEnvelope().getRawPayload());
            }
            if (result.getWeatherProductResult() != null && result.getWeatherProductResult().getProduct() != null) {
                weatherProducts.add(result.getWeatherProductResult().getProduct());
            }
            if (result.getPirepResult() != null && result.getPirepResult().getReport() != null) {
                pireps.add(result.getPirepResult().getReport());
            }
            if (result.getNotamRestriction() != null) {
                notams.add(result.getNotamRestriction());
            }
        }
    }

    private Map<String, String> persist(OperationalRuntimeRequest request,
                                        OperationalDecisionResult result) {
        Map<String, String> ids = new LinkedHashMap<>();
        OperationalDecisionRepository decisionRepository = request.getDecisionRepository();
        AuditReplayRepository auditRepository = request.getAuditReplayRepository();
        WeatherProductRepository weatherRepository = request.getWeatherProductRepository();
        PirepReportRepository pirepRepository = request.getPirepReportRepository();
        if (decisionRepository != null) {
            ids.put("decision", decisionRepository.save(result));
        }
        if (auditRepository != null && result.getAuditEnvelope() != null) {
            ids.put("audit", auditRepository.saveAuditEnvelope(result.getAuditEnvelope()));
        }
        if (auditRepository != null && result.getReplayBundle() != null) {
            ids.put("replay", auditRepository.saveReplayBundle(result.getReplayBundle()));
        }
        if (weatherRepository != null) {
            int i = 0;
            for (WeatherProduct product : result.getWeatherProducts()) {
                ids.put("weather." + i++, weatherRepository.save(product));
            }
        }
        if (pirepRepository != null) {
            int i = 0;
            for (org.tash.extensions.weather.pirep.PirepIngestResult pirep : result.getPirepResults()) {
                ids.put("pirep." + i++, pirepRepository.save(pirep));
            }
        }
        return ids;
    }
}
