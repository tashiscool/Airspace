package org.tash.extensions.messaging.transaction;

import org.tash.extensions.messaging.UsnsMessageEnvelope;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class UsnsTransactionSplitter {
    private static final Pattern MARKER = Pattern.compile(
            "(?im)(^\\s*!FDC\\b|^\\s*![A-Z0-9]+\\b|\\bNOTAM[NRJC]\\b|\\bSNOWTAM\\b|\\bBIRDTAM\\b|\\bASHTAM\\b|\\bGENOT\\s+RWA\\b|\\b(?:PIREP|UA|UUA|CONVECTIVE\\s+SIGMET|SIGMET|AIRMET|METAR|SPECI|TAF|NEXRAD|CWAP|CWAF|CWA)\\b|[() ]SVC\\s+(?:RQ|TBL)\\b|^\\s*(?:R|RGR)\\s+FDC\\s+\\d+/\\d+\\b)");

    public UsnsTransactionParseResult split(UsnsMessageEnvelope envelope) {
        List<String> warnings = new ArrayList<>();
        List<String> errors = new ArrayList<>();
        if (envelope == null || envelope.getBody() == null || envelope.getBody().trim().isEmpty()) {
            errors.add("Envelope body is empty");
            return result(false, Collections.emptyList(), warnings, errors);
        }

        List<Span> spans = spans(envelope.getBody());
        if (spans.isEmpty()) {
            UsnsTransaction transaction = transaction(UsnsTransactionType.ADMIN, envelope.getBody(), envelope, "No transaction marker found");
            return result(true, Collections.singletonList(transaction), warnings, errors);
        }

        List<UsnsTransaction> transactions = new ArrayList<>();
        for (int i = 0; i < spans.size(); i++) {
            int start = spans.get(i).start;
            int end = i + 1 < spans.size() ? spans.get(i + 1).start : envelope.getBody().length();
            String raw = envelope.getBody().substring(start, end).trim();
            UsnsTransactionType type = classify(raw, envelope);
            if (type == UsnsTransactionType.UNKNOWN) {
                errors.add("Unknown transaction type: " + preview(raw));
            }
            transactions.add(transaction(type, raw, envelope, null));
        }

        return result(errors.isEmpty(), transactions, warnings, errors);
    }

    private List<Span> spans(String body) {
        List<Span> spans = new ArrayList<>();
        Matcher matcher = MARKER.matcher(body);
        while (matcher.find()) {
            int start = matcher.start();
            if (!spans.isEmpty() && sameLine(body, spans.get(spans.size() - 1).start, start)) {
                continue;
            }
            spans.add(new Span(start));
        }
        spans.sort(Comparator.comparingInt(span -> span.start));
        List<Span> unique = new ArrayList<>();
        int last = -1;
        for (Span span : spans) {
            if (span.start != last) {
                unique.add(span);
                last = span.start;
            }
        }
        return unique;
    }

    private boolean sameLine(String body, int previousStart, int currentStart) {
        if (body == null || previousStart < 0 || currentStart <= previousStart) {
            return false;
        }
        String between = body.substring(previousStart, currentStart);
        return between.indexOf('\n') < 0 && between.indexOf('\r') < 0;
    }

    private UsnsTransaction transaction(UsnsTransactionType type, String raw, UsnsMessageEnvelope envelope, String warning) {
        List<String> warnings = new ArrayList<>();
        if (warning != null) {
            warnings.add(warning);
        }
        return UsnsTransaction.builder()
                .type(type)
                .rawText(raw)
                .normalizedText(normalize(raw))
                .envelope(envelope)
                .warnings(warnings)
                .build();
    }

    private UsnsTransactionType classify(String raw, UsnsMessageEnvelope envelope) {
        String text = normalize(raw).toUpperCase(Locale.US);
        if (text.startsWith("!FDC")) {
            return UsnsTransactionType.FDC;
        }
        if (text.startsWith("!")) {
            return UsnsTransactionType.DOMESTIC;
        }
        if (text.matches("^(R|RGR)\\s+FDC\\s+\\d+/\\d+\\b.*")) {
            return FdcAcknowledgementCommand.parse(text).isAccepted()
                    ? UsnsTransactionType.FDC_ACK
                    : UsnsTransactionType.UNKNOWN;
        }
        if (text.contains("SVC RQ") || text.startsWith("RQN")) {
            return UsnsTransactionType.SERVICE_REQUEST;
        }
        if (text.contains("SVC TBL")) {
            return UsnsTransactionType.SERVICE_TABLE;
        }
        if (text.contains("GENOT RWA")) {
            return UsnsTransactionType.GENOT;
        }
        if (text.startsWith("PIREP") || text.startsWith("UA ") || text.startsWith("UUA ")
                || text.contains(" PIREP ")) {
            return UsnsTransactionType.PIREP;
        }
        if (text.contains("CONVECTIVE SIGMET") || text.contains("SIGMET")) {
            return UsnsTransactionType.SIGMET;
        }
        if (text.contains("AIRMET")) {
            return UsnsTransactionType.AIRMET;
        }
        if (text.startsWith("METAR ") || text.contains(" METAR ")
                || text.startsWith("SPECI ") || text.contains(" SPECI ")) {
            return UsnsTransactionType.METAR;
        }
        if (text.startsWith("TAF ") || text.contains(" TAF ")) {
            return UsnsTransactionType.TAF;
        }
        if (text.contains("NEXRAD") || text.contains("CWAP") || text.contains("CWAF")) {
            return UsnsTransactionType.NEXRAD_ADVISORY;
        }
        if (text.startsWith("CWA ") || text.contains(" CWA ")
                || text.contains("WX ADVISORY") || text.contains("WEATHER ADVISORY")) {
            return UsnsTransactionType.WEATHER_ADVISORY;
        }
        if (text.contains("ASHTAM")) {
            return UsnsTransactionType.ASHTAM;
        }
        if (text.contains("SNOWTAM")) {
            return UsnsTransactionType.SNOWTAM;
        }
        if (text.contains("BIRDTAM")) {
            return UsnsTransactionType.BIRDTAM;
        }
        if (text.contains("NOTAMJ") || (text.contains("NOTAMN") && envelope != null
                && envelope.getOriginAddress() != null && envelope.getOriginAddress().startsWith("C")
                && startsWithNumber(text))) {
            return UsnsTransactionType.CANADIAN_DOMESTIC;
        }
        if (text.contains("NOTAMN")) {
            return UsnsTransactionType.ICAO_NOTAMN;
        }
        if (text.contains("NOTAMR")) {
            return UsnsTransactionType.ICAO_NOTAMR;
        }
        if (text.contains("NOTAMC")) {
            return UsnsTransactionType.ICAO_NOTAMC;
        }
        return UsnsTransactionType.UNKNOWN;
    }

    private boolean startsWithNumber(String text) {
        return text.matches("^\\d.*");
    }

    private String normalize(String text) {
        return text == null ? "" : text.replace('\r', '\n').replaceAll("\\s+", " ").trim();
    }

    private String preview(String raw) {
        String normalized = normalize(raw);
        return normalized.length() <= 40 ? normalized : normalized.substring(0, 40);
    }

    private UsnsTransactionParseResult result(boolean accepted, List<UsnsTransaction> transactions,
                                              List<String> warnings, List<String> errors) {
        return UsnsTransactionParseResult.builder()
                .accepted(accepted)
                .transactions(transactions)
                .warnings(warnings)
                .errors(errors)
                .build();
    }

    private static class Span {
        private final int start;

        private Span(int start) {
            this.start = start;
        }
    }
}
