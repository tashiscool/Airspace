package org.tash.extensions.messaging.transaction;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.messaging.UsnsMessageEnvelope;

import java.util.List;

@Data
@Builder
public class UsnsTransaction {
    private UsnsTransactionType type;
    private String rawText;
    private String normalizedText;
    private UsnsMessageEnvelope envelope;
    private List<String> warnings;
}
