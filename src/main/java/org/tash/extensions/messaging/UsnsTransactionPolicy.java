package org.tash.extensions.messaging;

import org.tash.extensions.messaging.transaction.UsnsTransaction;
import org.tash.extensions.messaging.transaction.UsnsTransactionType;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class UsnsTransactionPolicy {
    public static final int DEFAULT_NOTAM_MAX_SIZE = 1800;

    private final int notamMaxSize;

    public UsnsTransactionPolicy() {
        this(DEFAULT_NOTAM_MAX_SIZE);
    }

    public UsnsTransactionPolicy(int notamMaxSize) {
        this.notamMaxSize = notamMaxSize;
    }

    public UsnsRoutingOutcome evaluate(UsnsTransaction transaction) {
        List<String> warnings = new ArrayList<>();
        List<String> errors = new ArrayList<>();
        if (transaction == null) {
            errors.add("No USNS transaction supplied");
            return outcome(UsnsRoutingOutcomeType.PARSE_FAILURE, MrsFunctionCode.UNKNOWN, warnings, errors);
        }

        MrsHeader header = transaction.getEnvelope() == null ? null : transaction.getEnvelope().getMrsHeader();
        MrsFunctionCode function = header == null ? MrsFunctionCode.UNKNOWN : MrsFunctionCode.from(header.getFunctionCode());
        UsnsTransactionType type = transaction.getType();
        String raw = transaction.getRawText() == null ? "" : transaction.getRawText();

        if (type == UsnsTransactionType.ADMIN) {
            warnings.add("Message contains no transaction identifier");
            return outcome(UsnsRoutingOutcomeType.ADMIN_MESSAGE, function, warnings, errors);
        }
        if (type == UsnsTransactionType.UNKNOWN) {
            errors.add("Unknown transaction type");
            return outcome(UsnsRoutingOutcomeType.PARSE_FAILURE, function, warnings, errors);
        }

        if (type == UsnsTransactionType.FDC) {
            String origin = transaction.getEnvelope() == null ? "" : value(transaction.getEnvelope().getOriginAddress());
            if (!"KDZZNAXX".equals(origin)) {
                errors.add("FDC origin restricted to KDZZNAXX");
                return outcome(UsnsRoutingOutcomeType.BYPASS_ERQ, function, warnings, errors);
            }
            if (function == MrsFunctionCode.EDIT_NOTAM) {
                errors.add("FDC edits are not allowed with MRS function 07; use function 17 for comment edits");
                return outcome(UsnsRoutingOutcomeType.RETURN_TO_SENDER, function, warnings, errors);
            }
            if (function == MrsFunctionCode.FDC_COMMENTS_EDIT) {
                warnings.add("FDC comment edit recognized from MRS function 17");
            }
            enforceLength(raw, notamMaxSize - 8, "FDC text/comment", errors);
        }

        if (type == UsnsTransactionType.DOMESTIC) {
            enforceLength(raw, notamMaxSize - 7, "Domestic NOTAM text", errors);
        }

        if (transaction.getEnvelope() != null && transaction.getEnvelope().isWmscr()) {
            if (function == MrsFunctionCode.PRIVILEGED_REQUEST) {
                warnings.add("WMSCR privileged request");
            }
            if ("00".equals(header == null ? "" : header.getTerminalId())
                    && transaction.getEnvelope().getOriginLine() != null
                    && transaction.getEnvelope().getOriginLine().trim().endsWith("WMSCR")) {
                errors.add("WMSCR blank operator/terminal rejected");
                return outcome(UsnsRoutingOutcomeType.REJECTED, function, warnings, errors);
            }
        }

        if (!errors.isEmpty()) {
            return outcome(UsnsRoutingOutcomeType.REJECTED, function, warnings, errors);
        }
        return outcome(UsnsRoutingOutcomeType.ACCEPTED, function, warnings, errors);
    }

    private void enforceLength(String raw, int max, String label, List<String> errors) {
        if (raw.length() > max) {
            errors.add(label + " exceeds max circuit length " + max);
        }
    }

    private UsnsRoutingOutcome outcome(UsnsRoutingOutcomeType type,
                                       MrsFunctionCode function,
                                       List<String> warnings,
                                       List<String> errors) {
        return UsnsRoutingOutcome.builder()
                .type(type)
                .functionCode(function)
                .accepted(type == UsnsRoutingOutcomeType.ACCEPTED
                        || type == UsnsRoutingOutcomeType.ADMIN_MESSAGE
                        || type == UsnsRoutingOutcomeType.CLASSIFIED_ONLY)
                .returnToSender(type == UsnsRoutingOutcomeType.RETURN_TO_SENDER)
                .bypassErq(type == UsnsRoutingOutcomeType.BYPASS_ERQ)
                .privilegedRequest(function == MrsFunctionCode.PRIVILEGED_REQUEST)
                .warnings(Collections.unmodifiableList(new ArrayList<>(warnings)))
                .errors(Collections.unmodifiableList(new ArrayList<>(errors)))
                .build();
    }

    private String value(String value) {
        return value == null ? "" : value.trim().toUpperCase();
    }
}
