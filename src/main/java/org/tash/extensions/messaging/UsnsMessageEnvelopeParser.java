package org.tash.extensions.messaging;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class UsnsMessageEnvelopeParser {
    public UsnsMessageParseResult parse(String raw) {
        String normalized = MessageControlCharacters.normalizeLineEndings(raw);
        if (looksLikeWmscr(normalized)) {
            return new WmscrMessageParser().parse(raw);
        }
        UsnsMessageParseResult nadin = new NadinMessageParser().parse(raw);
        if (nadin.isAccepted() && isUsnofAddressed(nadin.getEnvelope())) {
            return new UsnofMessageParser().parse(raw);
        }
        return nadin;
    }

    private boolean looksLikeWmscr(String text) {
        return text.contains("WMSCR")
                || text.contains("90006")
                || text.contains("90008")
                || text.startsWith("01GGCE");
    }

    private boolean isUsnofAddressed(UsnsMessageEnvelope envelope) {
        if (envelope == null || envelope.getToAddresses() == null) {
            return false;
        }
        return envelope.getToAddresses().stream()
                .anyMatch(address -> Arrays.asList("KDZZNAXX", "KDZZNAXXRR", "KCNFYNYX", "KDCAYNYX").contains(address));
    }

    protected UsnsMessageParseResult parseNadinLike(String raw, boolean wmscr, boolean usnof) {
        List<String> warnings = new ArrayList<>();
        List<String> errors = new ArrayList<>();
        String normalized = MessageControlCharacters.normalizeLineEndings(raw);
        String[] lines = normalized.split("\n", -1);
        if (lines.length == 0 || lines[0].trim().isEmpty()) {
            errors.add("MRS header is missing");
            return rejected(errors, warnings);
        }

        String firstLine = lines[0];
        String headingFromHeader = null;
        int embeddedHeading = firstLine.indexOf(MessageControlCharacters.SOH);
        if (embeddedHeading >= 0) {
            headingFromHeader = firstLine.substring(embeddedHeading);
            firstLine = firstLine.substring(0, embeddedHeading);
        }
        MrsHeader header = MrsHeader.parse(firstLine);
        if (!header.isValid()) {
            errors.add(header.getError() + ": " + firstLine);
        }

        int index = 1;
        String heading = headingFromHeader;
        if (heading == null && index < lines.length) {
            heading = lines[index++];
        }
        heading = MessageControlCharacters.stripLeadingControls(heading);

        String addressLine = index < lines.length ? lines[index++].trim() : "";
        StringBuilder addressBuilder = new StringBuilder(addressLine);
        while (index < lines.length && isAdditionalAddressLine(lines[index]) && !containsStx(lines[index])) {
            String line = lines[index].trim();
            if (!line.isEmpty()) {
                addressBuilder.append("\n").append(line);
            }
            index++;
        }
        addressLine = addressBuilder.toString();

        String originLine = "";
        if (index < lines.length && !containsStx(lines[index])) {
            originLine = lines[index++].trim();
        }

        StringBuilder body = new StringBuilder();
        while (index < lines.length) {
            if (body.length() > 0) {
                body.append('\n');
            }
            body.append(lines[index++]);
        }
        String bodyText = MessageControlCharacters.stripBodyTerminators(body.toString());
        int stx = bodyText.indexOf(MessageControlCharacters.STX);
        if (stx >= 0) {
            bodyText = bodyText.substring(stx + 1).trim();
        }

        String priority = firstToken(addressLine);
        List<String> toAddresses = tokensAfterFirst(addressLine.replace('\n', ' '));
        String originDhm = firstToken(originLine);
        String originAddress = secondToken(originLine);
        if (!originDhm.isEmpty() && !originDhm.matches("\\d{6}")) {
            errors.add("Origin date-time group must be DDHHMM: " + originDhm);
        }
        if (bodyText.isEmpty()) {
            warnings.add("Message text is missing");
        }

        UsnsMessageEnvelope envelope = UsnsMessageEnvelope.builder()
                .rawText(raw)
                .mrsHeader(header)
                .heading(heading)
                .addressLine(addressLine)
                .originLine(originLine)
                .originDhm(originDhm)
                .originAddress(originAddress)
                .priority(priority)
                .toAddresses(toAddresses)
                .body(bodyText)
                .wmscr(wmscr)
                .usnof(usnof)
                .build();

        UsnsRoutingDecision routing = routingDecision(envelope, heading, bodyText, wmscr, usnof);
        boolean accepted = errors.isEmpty() && routing != UsnsRoutingDecision.REJECTED;
        return UsnsMessageParseResult.builder()
                .accepted(accepted)
                .envelope(envelope)
                .warnings(warnings)
                .errors(errors)
                .routingDecision(routing)
                .normalizedBody(bodyText)
                .build();
    }

    private UsnsRoutingDecision routingDecision(UsnsMessageEnvelope envelope, String heading, String body, boolean wmscr, boolean usnof) {
        String upperHeading = heading == null ? "" : heading.toUpperCase();
        String upperBody = body == null ? "" : body.toUpperCase();
        if (wmscr && upperHeading.contains("90008")) {
            return UsnsRoutingDecision.REJECTED;
        }
        if (wmscr && upperHeading.contains("90006")) {
            return UsnsRoutingDecision.PRIVILEGED_REQUEST;
        }
        if (upperBody.contains("NAT TRACK") || upperBody.contains("NORTH ATLANTIC ADVISORY")) {
            return UsnsRoutingDecision.NAT_TRACK;
        }
        if (usnof) {
            if (envelope.getToAddresses().contains("KDCAYNYX")) {
                return UsnsRoutingDecision.ADMIN_MESSAGE;
            }
            if (envelope.getToAddresses().stream().anyMatch(a -> a.equals("KDZZNAXX") || a.equals("KDZZNAXXRR") || a.equals("KCNFYNYX"))) {
                return UsnsRoutingDecision.ACCEPT_USNS;
            }
            return UsnsRoutingDecision.NOT_ADDRESSED_TO_USNS;
        }
        return UsnsRoutingDecision.UNKNOWN;
    }

    private UsnsMessageParseResult rejected(List<String> errors, List<String> warnings) {
        return UsnsMessageParseResult.builder()
                .accepted(false)
                .warnings(warnings)
                .errors(errors)
                .routingDecision(UsnsRoutingDecision.REJECTED)
                .normalizedBody("")
                .build();
    }

    private boolean startsWithDhm(String line) {
        return line != null && line.trim().matches("\\d{6}\\s+.*");
    }

    private boolean isAdditionalAddressLine(String line) {
        if (line == null || line.trim().isEmpty() || startsWithDhm(line)) {
            return false;
        }
        String first = firstToken(line);
        return first.matches("SS|DD|FF|GG");
    }

    private boolean containsStx(String line) {
        return line != null && line.indexOf(MessageControlCharacters.STX) >= 0;
    }

    private String firstToken(String value) {
        if (value == null || value.trim().isEmpty()) {
            return "";
        }
        return value.trim().split("\\s+")[0];
    }

    private String secondToken(String value) {
        if (value == null || value.trim().isEmpty()) {
            return "";
        }
        String[] tokens = value.trim().split("\\s+");
        return tokens.length > 1 ? tokens[1] : "";
    }

    private List<String> tokensAfterFirst(String value) {
        if (value == null || value.trim().isEmpty()) {
            return Collections.emptyList();
        }
        String[] tokens = value.trim().split("\\s+");
        if (tokens.length <= 1) {
            return Collections.emptyList();
        }
        return new ArrayList<>(Arrays.asList(tokens).subList(1, tokens.length));
    }
}
