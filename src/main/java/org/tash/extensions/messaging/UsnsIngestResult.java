package org.tash.extensions.messaging;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.messaging.transaction.UsnsTransaction;
import org.tash.extensions.messaging.transaction.UsnsTransactionParseResult;
import org.tash.extensions.carf.api.CarfAnalysisResult;
import org.tash.extensions.notam.DomesticNotamParseResult;
import org.tash.extensions.notam.NotamFieldParseResult;
import org.tash.extensions.reservation.CarfRouteMessage;
import org.tash.extensions.weather.pirep.PirepIngestResult;
import org.tash.extensions.weather.product.WeatherProduct;
import org.tash.extensions.weather.product.WeatherProductParseResult;

import java.util.List;
import java.util.stream.Collectors;

@Data
@Builder
public class UsnsIngestResult {
    private UsnsMessageParseResult envelopeResult;
    private UsnsTransactionParseResult transactionResults;
    private List<UsnsTransactionIngestResult> transactionIngestResults;
    private List<String> errors;

    public boolean isAccepted() {
        return (envelopeResult == null || envelopeResult.isAccepted())
                && (transactionResults == null || transactionResults.isAccepted())
                && (errors == null || errors.isEmpty())
                && getRejectedWeatherResults().isEmpty();
    }

    public List<DomesticNotamParseResult> getDomesticResults() {
        return transactionIngestResults == null ? java.util.Collections.emptyList() : transactionIngestResults.stream()
                .map(UsnsTransactionIngestResult::getDomesticResult)
                .filter(java.util.Objects::nonNull)
                .collect(Collectors.toList());
    }

    public List<NotamFieldParseResult> getNotamFieldResults() {
        return transactionIngestResults == null ? java.util.Collections.emptyList() : transactionIngestResults.stream()
                .map(UsnsTransactionIngestResult::getNotamFieldResult)
                .filter(java.util.Objects::nonNull)
                .collect(Collectors.toList());
    }

    public List<CarfRouteMessage> getCarfMessages() {
        return transactionIngestResults == null ? java.util.Collections.emptyList() : transactionIngestResults.stream()
                .map(UsnsTransactionIngestResult::getCarfMessage)
                .filter(java.util.Objects::nonNull)
                .collect(Collectors.toList());
    }

    public List<CarfAnalysisResult> getCarfAnalysisResults() {
        return transactionIngestResults == null ? java.util.Collections.emptyList() : transactionIngestResults.stream()
                .map(UsnsTransactionIngestResult::getCarfAnalysisResult)
                .filter(java.util.Objects::nonNull)
                .collect(Collectors.toList());
    }

    public List<UsnsTransaction> getUnsupportedTransactions() {
        return transactionIngestResults == null ? java.util.Collections.emptyList() : transactionIngestResults.stream()
                .filter(result -> !result.isSupported())
                .map(UsnsTransactionIngestResult::getTransaction)
                .collect(Collectors.toList());
    }

    public List<CarfMessageFamilyParseResult> getFamilyParseResults() {
        return transactionIngestResults == null ? java.util.Collections.emptyList() : transactionIngestResults.stream()
                .map(UsnsTransactionIngestResult::getFamilyParseResult)
                .filter(java.util.Objects::nonNull)
                .collect(Collectors.toList());
    }

    public List<WeatherProduct> getWeatherProducts() {
        return transactionIngestResults == null ? java.util.Collections.emptyList() : transactionIngestResults.stream()
                .map(UsnsTransactionIngestResult::getWeatherProductResult)
                .filter(java.util.Objects::nonNull)
                .map(WeatherProductParseResult::getProduct)
                .filter(java.util.Objects::nonNull)
                .collect(Collectors.toList());
    }

    public List<WeatherProductParseResult> getWeatherProductResults() {
        return transactionIngestResults == null ? java.util.Collections.emptyList() : transactionIngestResults.stream()
                .map(UsnsTransactionIngestResult::getWeatherProductResult)
                .filter(java.util.Objects::nonNull)
                .collect(Collectors.toList());
    }

    public List<WeatherProductParseResult> getClassifiedOnlyWeatherResults() {
        return getWeatherProductResults().stream()
                .filter(WeatherProductParseResult::isClassifiedOnly)
                .collect(Collectors.toList());
    }

    public List<WeatherProductParseResult> getRejectedWeatherResults() {
        return getWeatherProductResults().stream()
                .filter(result -> !result.isAccepted() && !result.isClassifiedOnly())
                .collect(Collectors.toList());
    }

    public List<PirepIngestResult> getPirepResults() {
        return transactionIngestResults == null ? java.util.Collections.emptyList() : transactionIngestResults.stream()
                .map(UsnsTransactionIngestResult::getPirepIngestResult)
                .filter(java.util.Objects::nonNull)
                .collect(Collectors.toList());
    }
}
