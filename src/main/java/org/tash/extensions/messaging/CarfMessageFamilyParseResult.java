package org.tash.extensions.messaging;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.messaging.transaction.UsnsTransactionType;

import java.util.List;
import java.util.Map;

@Data
@Builder
public class CarfMessageFamilyParseResult {
    private UsnsTransactionType type;
    private CarfMessageFamilyStatus status;
    private String rawText;
    private Map<String, String> fields;
    private String semantic;
    private List<String> warnings;
    private List<String> errors;

    public boolean isAccepted() {
        return status != CarfMessageFamilyStatus.REJECTED;
    }
}
