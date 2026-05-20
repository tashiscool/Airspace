package org.tash.extensions.feed;

import org.tash.extensions.carf.api.CarfAnalysisResult;
import org.tash.extensions.carf.api.CarfAnalysisService;
import org.tash.extensions.messaging.UsnsIngestResult;
import org.tash.extensions.messaging.UsnsIngestService;
import org.tash.extensions.notam.NotamAirspaceParser;
import org.tash.extensions.notam.NotamAirspaceRestriction;
import org.tash.extensions.weather.pirep.PirepIngestResult;
import org.tash.extensions.weather.pirep.PirepIngestService;
import org.tash.extensions.weather.product.WeatherProductParseResult;
import org.tash.extensions.weather.product.WeatherProductParser;
import org.tash.extensions.weather.product.WeatherProductType;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

public class OperationalFeedIngestService {
    private final UsnsIngestService usnsIngestService;
    private final CarfAnalysisService carfAnalysisService;
    private final WeatherProductParser weatherProductParser;
    private final PirepIngestService pirepIngestService;
    private final NotamAirspaceParser notamAirspaceParser;

    public OperationalFeedIngestService() {
        this(new UsnsIngestService(), new CarfAnalysisService(), new WeatherProductParser(),
                new PirepIngestService(), new NotamAirspaceParser());
    }

    public OperationalFeedIngestService(UsnsIngestService usnsIngestService,
                                        CarfAnalysisService carfAnalysisService,
                                        WeatherProductParser weatherProductParser,
                                        PirepIngestService pirepIngestService,
                                        NotamAirspaceParser notamAirspaceParser) {
        this.usnsIngestService = usnsIngestService == null ? new UsnsIngestService() : usnsIngestService;
        this.carfAnalysisService = carfAnalysisService == null ? new CarfAnalysisService() : carfAnalysisService;
        this.weatherProductParser = weatherProductParser == null ? new WeatherProductParser() : weatherProductParser;
        this.pirepIngestService = pirepIngestService == null ? new PirepIngestService() : pirepIngestService;
        this.notamAirspaceParser = notamAirspaceParser == null ? new NotamAirspaceParser() : notamAirspaceParser;
    }

    public OperationalFeedBatchResult ingest(OperationalFeedPollResult pollResult) {
        List<OperationalFeedIngestResult> results = new ArrayList<>();
        List<String> diagnostics = new ArrayList<>();
        if (pollResult == null) {
            diagnostics.add("Feed poll result is missing");
            return OperationalFeedBatchResult.builder()
                    .receivedAt(ZonedDateTime.now(ZoneOffset.UTC))
                    .diagnostics(diagnostics)
                    .build();
        }
        diagnostics.addAll(pollResult.getDiagnostics());
        String sourceId = null;
        for (OperationalFeedEnvelope envelope : pollResult.getEnvelopes()) {
            if (sourceId == null && envelope != null) {
                sourceId = envelope.getSourceId();
            }
            results.add(ingest(envelope));
        }
        return OperationalFeedBatchResult.builder()
                .sourceId(sourceId)
                .receivedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .results(results)
                .diagnostics(diagnostics)
                .build();
    }

    public OperationalFeedIngestResult ingest(OperationalFeedEnvelope envelope) {
        List<String> warnings = new ArrayList<>();
        List<String> errors = new ArrayList<>();
        List<String> artifacts = new ArrayList<>();
        if (envelope == null) {
            errors.add("Feed envelope is missing");
            return OperationalFeedIngestResult.builder().accepted(false).errors(errors).build();
        }
        String raw = envelope.getRawPayload();
        String hash = envelope.payloadHash();
        OperationalFeedType type = envelope.getType() == null ? OperationalFeedType.UNKNOWN : envelope.getType();
        try {
            switch (type) {
                case USNS:
                    UsnsIngestResult usns = usnsIngestService.parse(raw);
                    artifacts.add("usns:" + envelope.getId());
                    usns.getWeatherProducts().forEach(product -> artifacts.add("weather:" + product.getId()));
                    usns.getPirepResults().forEach(pirep -> artifacts.add("pirep:" + idOf(pirep)));
                    usns.getCarfAnalysisResults().forEach(carf -> artifacts.add("carf:" + artifacts.size()));
                    errors.addAll(usns.getErrors() == null ? java.util.Collections.emptyList() : usns.getErrors());
                    return result(envelope, usns.isAccepted(), hash, artifacts, warnings, errors)
                            .usnsResult(usns).build();
                case CARF_ALTRV:
                    CarfAnalysisResult carf = carfAnalysisService.parseValidateMap(raw);
                    artifacts.add("carf:" + envelope.getId());
                    warnings.addAll(carf.getDiagnostics() == null ? java.util.Collections.emptyList() : carf.getDiagnostics());
                    return result(envelope, carf.isAccepted(), hash, artifacts, warnings, errors)
                            .carfResult(carf).build();
                case PIREP:
                    WeatherProductParseResult pirepParse = weatherProductParser.parse(raw, WeatherProductType.PIREP_DERIVED);
                    warnings.addAll(pirepParse.getWarnings());
                    errors.addAll(pirepParse.getErrors());
                    PirepIngestResult pirep = pirepParse.getPirepReport() == null
                            ? null
                            : pirepIngestService.ingest(pirepParse.getPirepReport(), null);
                    if (pirep != null) {
                        artifacts.add("pirep:" + idOf(pirep));
                    }
                    return result(envelope, pirep != null && pirep.isAccepted(), hash, artifacts, warnings, errors)
                            .weatherProductResult(pirepParse)
                            .pirepResult(pirep)
                            .build();
                case WEATHER:
                    WeatherProductParseResult weather = weatherProductParser.parse(raw, null);
                    warnings.addAll(weather.getWarnings());
                    errors.addAll(weather.getErrors());
                    if (weather.getProduct() != null) {
                        artifacts.add("weather:" + weather.getProduct().getId());
                    }
                    return result(envelope, weather.isAccepted() || weather.isClassifiedOnly(), hash, artifacts, warnings, errors)
                            .weatherProductResult(weather)
                            .build();
                case NOTAM:
                    NotamAirspaceRestriction notam = notamAirspaceParser.parse(raw);
                    artifacts.add("notam:" + notam.getId());
                    return result(envelope, true, hash, artifacts, warnings, errors)
                            .notamRestriction(notam)
                            .build();
                default:
                    WeatherProductParseResult retained = weatherProductParser.parse(raw, null);
                    warnings.add("Unknown feed type retained after best-effort weather classification");
                    warnings.addAll(retained.getWarnings());
                    errors.addAll(retained.getErrors());
                    return result(envelope, retained.isAccepted() || retained.isClassifiedOnly(), hash, artifacts, warnings, errors)
                            .weatherProductResult(retained)
                            .build();
            }
        } catch (RuntimeException ex) {
            errors.add(ex.getMessage());
            return result(envelope, false, hash, artifacts, warnings, errors).build();
        }
    }

    private OperationalFeedIngestResult.OperationalFeedIngestResultBuilder result(OperationalFeedEnvelope envelope,
                                                                                  boolean accepted,
                                                                                  String hash,
                                                                                  List<String> artifacts,
                                                                                  List<String> warnings,
                                                                                  List<String> errors) {
        return OperationalFeedIngestResult.builder()
                .envelope(envelope)
                .accepted(accepted && errors.isEmpty())
                .rawPayloadHash(hash)
                .downstreamArtifactIds(new ArrayList<>(artifacts))
                .warnings(new ArrayList<>(warnings))
                .errors(new ArrayList<>(errors));
    }

    private String idOf(PirepIngestResult result) {
        return result == null || result.getReport() == null ? "unknown" : result.getReport().getId();
    }
}
