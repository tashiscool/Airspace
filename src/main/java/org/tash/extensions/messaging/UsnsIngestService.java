package org.tash.extensions.messaging;

import org.tash.extensions.messaging.transaction.FdcAcknowledgementCommand;
import org.tash.extensions.messaging.transaction.ServiceRequestCommand;
import org.tash.extensions.messaging.transaction.ServiceTableCommand;
import org.tash.extensions.messaging.transaction.UsnsTransaction;
import org.tash.extensions.messaging.transaction.UsnsTransactionParseResult;
import org.tash.extensions.messaging.transaction.UsnsTransactionSplitter;
import org.tash.extensions.messaging.transaction.UsnsTransactionType;
import org.tash.extensions.carf.api.CarfAnalysisResult;
import org.tash.extensions.carf.api.CarfAnalysisService;
import org.tash.extensions.notam.DomesticNotamParseResult;
import org.tash.extensions.notam.DomesticNotamParser;
import org.tash.extensions.notam.FdcLaserAirspaceParser;
import org.tash.extensions.notam.NotamAirspaceRestriction;
import org.tash.extensions.notam.NotamAirspaceParser;
import org.tash.extensions.notam.NotamFieldParseResult;
import org.tash.extensions.reservation.CarfRouteMessage;
import org.tash.extensions.reservation.CarfRouteMessageParser;
import org.tash.extensions.weather.pirep.PirepIngestResult;
import org.tash.extensions.weather.pirep.PirepIngestService;
import org.tash.extensions.weather.product.WeatherProductParseResult;
import org.tash.extensions.weather.product.WeatherProductParser;
import org.tash.extensions.weather.product.WeatherProductType;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class UsnsIngestService {
    private final UsnsMessageEnvelopeParser envelopeParser;
    private final UsnsTransactionSplitter transactionSplitter;
    private final DomesticNotamParser domesticNotamParser;
    private final CarfRouteMessageParser carfRouteMessageParser;
    private final CarfAnalysisService carfAnalysisService;
    private final FdcLaserAirspaceParser fdcLaserAirspaceParser;
    private final NotamAirspaceParser notamAirspaceParser = new NotamAirspaceParser();
    private final CarfMessageFamilyParser familyParser = new CarfMessageFamilyParser();
    private final UsnsTransactionPolicy transactionPolicy = new UsnsTransactionPolicy();
    private final WeatherProductParser weatherProductParser = new WeatherProductParser();
    private final PirepIngestService pirepIngestService = new PirepIngestService();

    public UsnsIngestService() {
        this(new UsnsMessageEnvelopeParser(), new UsnsTransactionSplitter(), new DomesticNotamParser(),
                new CarfRouteMessageParser(), new CarfAnalysisService(), new FdcLaserAirspaceParser());
    }

    public UsnsIngestService(UsnsMessageEnvelopeParser envelopeParser,
                             UsnsTransactionSplitter transactionSplitter,
                             DomesticNotamParser domesticNotamParser,
                             CarfRouteMessageParser carfRouteMessageParser,
                             CarfAnalysisService carfAnalysisService,
                             FdcLaserAirspaceParser fdcLaserAirspaceParser) {
        this.envelopeParser = envelopeParser;
        this.transactionSplitter = transactionSplitter;
        this.domesticNotamParser = domesticNotamParser;
        this.carfRouteMessageParser = carfRouteMessageParser;
        this.carfAnalysisService = carfAnalysisService;
        this.fdcLaserAirspaceParser = fdcLaserAirspaceParser;
    }

    public UsnsIngestResult parse(String raw) {
        UsnsMessageParseResult envelopeResult = envelopeParser.parse(raw);
        List<String> errors = new ArrayList<>();
        if (!envelopeResult.isAccepted()) {
            errors.addAll(envelopeResult.getErrors());
            return UsnsIngestResult.builder()
                    .envelopeResult(envelopeResult)
                    .transactionResults(emptyTransactionResult(false, errors))
                    .transactionIngestResults(Collections.emptyList())
                    .errors(errors)
                    .build();
        }

        UsnsTransactionParseResult transactionResults = transactionSplitter.split(envelopeResult.getEnvelope());
        if (!transactionResults.isAccepted()) {
            errors.addAll(transactionResults.getErrors());
            return UsnsIngestResult.builder()
                    .envelopeResult(envelopeResult)
                    .transactionResults(transactionResults)
                    .transactionIngestResults(Collections.emptyList())
                    .errors(errors)
                    .build();
        }

        List<UsnsTransactionIngestResult> ingested = new ArrayList<>();
        for (UsnsTransaction transaction : transactionResults.getTransactions()) {
            ingested.add(ingest(transaction));
        }
        for (UsnsTransactionIngestResult result : ingested) {
            errors.addAll(result.getErrors());
        }

        return UsnsIngestResult.builder()
                .envelopeResult(envelopeResult)
                .transactionResults(transactionResults)
                .transactionIngestResults(ingested)
                .errors(errors)
                .build();
    }

    private UsnsTransactionIngestResult ingest(UsnsTransaction transaction) {
        List<String> warnings = new ArrayList<>();
        List<String> errors = new ArrayList<>();
        try {
            UsnsRoutingOutcome routingOutcome = transactionPolicy.evaluate(transaction);
            warnings.addAll(routingOutcome.getWarnings());
            errors.addAll(routingOutcome.getErrors());
            if (!routingOutcome.isAccepted()) {
                return UsnsTransactionIngestResult.builder()
                        .transaction(transaction)
                        .supported(false)
                        .routingOutcome(routingOutcome)
                        .warnings(warnings)
                        .errors(errors)
                        .build();
            }
            if (isCarfRoute(transaction.getRawText())) {
                CarfRouteMessage message = carfRouteMessageParser.parse(transaction.getRawText());
                CarfAnalysisResult analysisResult = carfAnalysisService.parseAndMap(transaction.getRawText());
                if (!analysisResult.isAccepted()) {
                    warnings.addAll(analysisResult.getDiagnostics());
                }
                return supported(transaction)
                        .routingOutcome(routingOutcome)
                        .carfMessage(message)
                        .carfAnalysisResult(analysisResult)
                        .warnings(warnings)
                        .errors(errors)
                        .build();
            }
            if (transaction.getType() == UsnsTransactionType.DOMESTIC) {
                DomesticNotamParseResult result = domesticNotamParser.parseDetailed(transaction.getRawText());
                if (!result.isAccepted()) {
                    errors.add(result.getRejectionReason());
                }
                return supported(transaction).routingOutcome(routingOutcome)
                        .domesticResult(result).warnings(warnings).errors(errors).build();
            }
            if (transaction.getType() == UsnsTransactionType.FDC) {
                try {
                    NotamAirspaceRestriction restriction = fdcLaserAirspaceParser.parse(transaction.getRawText());
                    return supported(transaction).routingOutcome(routingOutcome)
                            .fdcLaserRestriction(restriction).warnings(warnings).errors(errors).build();
                } catch (IllegalArgumentException ex) {
                    warnings.add("FDC transaction is classified but not an FDC laser restriction: " + ex.getMessage());
                    return unsupported(transaction, routingOutcome, warnings, errors);
                }
            }
            if (transaction.getType() == UsnsTransactionType.FDC_ACK) {
                FdcAcknowledgementCommand ack = FdcAcknowledgementCommand.parse(transaction.getRawText());
                if (!ack.isAccepted()) {
                    errors.add(ack.getRejectionReason());
                }
                return supported(transaction).routingOutcome(routingOutcome)
                        .fdcAcknowledgement(ack).warnings(warnings).errors(errors).build();
            }
            if (isIcaoOrCanadianNotam(transaction.getType())) {
                NotamFieldParseResult fields = notamAirspaceParser.parseFields(transaction.getRawText());
                warnings.addAll(fields.getWarnings());
                warnings.addAll(fields.getDiagnostics());
                return supported(transaction)
                        .routingOutcome(routingOutcome)
                        .notamFieldResult(fields)
                        .warnings(warnings)
                        .errors(errors)
                        .build();
            }
            if (transaction.getType() == UsnsTransactionType.SERVICE_REQUEST) {
                ServiceRequestCommand command = ServiceRequestCommand.parse(transaction.getRawText());
                warnings.addAll(command.getWarnings() == null ? java.util.Collections.emptyList() : command.getWarnings());
                errors.addAll(command.getErrors() == null ? java.util.Collections.emptyList() : command.getErrors());
                return supported(transaction).serviceRequest(command)
                        .routingOutcome(routingOutcome).warnings(warnings).errors(errors).build();
            }
            if (transaction.getType() == UsnsTransactionType.SERVICE_TABLE) {
                ServiceTableCommand command = ServiceTableCommand.parse(transaction.getRawText());
                warnings.addAll(command.getWarnings() == null ? java.util.Collections.emptyList() : command.getWarnings());
                errors.addAll(command.getErrors() == null ? java.util.Collections.emptyList() : command.getErrors());
                return supported(transaction).serviceTable(command)
                        .routingOutcome(routingOutcome).warnings(warnings).errors(errors).build();
            }
            if (isWeatherTransaction(transaction.getType())) {
                WeatherProductParseResult weatherResult = weatherProductParser.parse(transaction.getRawText(),
                        weatherHint(transaction.getType()));
                warnings.addAll(weatherResult.getWarnings());
                errors.addAll(weatherResult.getErrors());
                PirepIngestResult pirepResult = weatherResult.getPirepReport() == null
                        ? null
                        : pirepIngestService.ingest(weatherResult.getPirepReport(), null);
                if (pirepResult != null) {
                    pirepResult.getDiagnostics().forEach(d -> warnings.add(d.getMessage()));
                    if (!pirepResult.isAccepted()) {
                        errors.add("PIREP rejected: " + pirepResult.getDiagnostics());
                    }
                }
                return supported(transaction)
                        .routingOutcome(routingOutcome)
                        .weatherProductResult(weatherResult)
                        .pirepIngestResult(pirepResult)
                        .warnings(warnings)
                        .errors(errors)
                        .build();
            }
            warnings.add("Unsupported but classified transaction: " + transaction.getType());
            CarfMessageFamilyParseResult familyResult = familyParser.parse(transaction);
            warnings.addAll(familyResult.getWarnings());
            errors.addAll(familyResult.getErrors());
            return UsnsTransactionIngestResult.builder()
                    .transaction(transaction)
                    .supported(false)
                    .routingOutcome(routingOutcome)
                    .warnings(warnings)
                    .errors(errors)
                    .familyParseResult(familyResult)
                    .build();
        } catch (RuntimeException ex) {
            errors.add(ex.getMessage());
            return unsupported(transaction, null, warnings, errors);
        }
    }

    private boolean isCarfRoute(String text) {
        String normalized = text == null ? "" : text.replace("\r", "");
        return normalized.matches("(?s).*\\n?A\\.\\s+.*")
                && normalized.matches("(?s).*\\nD\\.\\s+.*")
                && normalized.matches("(?s).*\\nF\\.\\s+.*");
    }

    private boolean isWeatherTransaction(UsnsTransactionType type) {
        return type == UsnsTransactionType.PIREP
                || type == UsnsTransactionType.SIGMET
                || type == UsnsTransactionType.AIRMET
                || type == UsnsTransactionType.METAR
                || type == UsnsTransactionType.TAF
                || type == UsnsTransactionType.NEXRAD_ADVISORY
                || type == UsnsTransactionType.WEATHER_ADVISORY;
    }

    private boolean isIcaoOrCanadianNotam(UsnsTransactionType type) {
        return type == UsnsTransactionType.ICAO_NOTAMN
                || type == UsnsTransactionType.ICAO_NOTAMR
                || type == UsnsTransactionType.ICAO_NOTAMC
                || type == UsnsTransactionType.CANADIAN_DOMESTIC;
    }

    private WeatherProductType weatherHint(UsnsTransactionType type) {
        switch (type) {
            case PIREP:
                return WeatherProductType.PIREP_DERIVED;
            case SIGMET:
                return WeatherProductType.SIGMET;
            case AIRMET:
                return WeatherProductType.AIRMET;
            case METAR:
                return WeatherProductType.METAR;
            case TAF:
                return WeatherProductType.TAF;
            case NEXRAD_ADVISORY:
                return WeatherProductType.NEXRAD_POLYGON;
            default:
                return WeatherProductType.GENERIC_FORECAST_HAZARD;
        }
    }

    private UsnsTransactionIngestResult.UsnsTransactionIngestResultBuilder supported(UsnsTransaction transaction) {
        return UsnsTransactionIngestResult.builder().transaction(transaction).supported(true);
    }

    private UsnsTransactionIngestResult unsupported(UsnsTransaction transaction,
                                                   UsnsRoutingOutcome routingOutcome,
                                                   List<String> warnings,
                                                   List<String> errors) {
        return UsnsTransactionIngestResult.builder()
                .transaction(transaction)
                .supported(false)
                .routingOutcome(routingOutcome)
                .warnings(warnings)
                .errors(errors)
                .build();
    }

    private UsnsTransactionParseResult emptyTransactionResult(boolean accepted, List<String> errors) {
        return UsnsTransactionParseResult.builder()
                .accepted(accepted)
                .transactions(Collections.emptyList())
                .warnings(Collections.emptyList())
                .errors(errors)
                .build();
    }
}
