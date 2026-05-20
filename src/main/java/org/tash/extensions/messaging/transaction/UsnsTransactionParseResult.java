package org.tash.extensions.messaging.transaction;

import lombok.Builder;
import lombok.Data;

import java.util.List;

@Data
@Builder
public class UsnsTransactionParseResult {
    private boolean accepted;
    private List<UsnsTransaction> transactions;
    private List<String> warnings;
    private List<String> errors;
}
