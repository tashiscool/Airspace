package org.tash.extensions.messaging;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.messaging.transaction.UsnsTransaction;
import org.tash.extensions.messaging.transaction.UsnsTransactionParseResult;
import org.tash.extensions.carf.api.CarfAnalysisResult;
import org.tash.extensions.notam.DomesticNotamParseResult;
import org.tash.extensions.reservation.CarfRouteMessage;

import java.util.List;
import java.util.stream.Collectors;

@Data
@Builder
public class UsnsIngestResult {
    private UsnsMessageParseResult envelopeResult;
    private UsnsTransactionParseResult transactionResults;
    private List<UsnsTransactionIngestResult> transactionIngestResults;
    private List<String> errors;

    public List<DomesticNotamParseResult> getDomesticResults() {
        return transactionIngestResults == null ? java.util.Collections.emptyList() : transactionIngestResults.stream()
                .map(UsnsTransactionIngestResult::getDomesticResult)
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
}
