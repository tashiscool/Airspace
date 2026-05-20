package org.tash.extensions.messaging.transaction;

import lombok.Builder;
import lombok.Data;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

@Data
@Builder
public class FdcAcknowledgementCommand {
    private boolean accepted;
    private String notamId;
    private String initials;
    private String rejectionReason;

    private static final Pattern ACK =
            Pattern.compile("^\\s*(?:R|RGR)\\s+FDC\\s+(\\d+/\\d+)\\s+([A-Z]+)\\b", Pattern.CASE_INSENSITIVE);

    public static FdcAcknowledgementCommand parse(String text) {
        Matcher matcher = ACK.matcher(text == null ? "" : text.trim());
        if (!matcher.find()) {
            return FdcAcknowledgementCommand.builder()
                    .accepted(false)
                    .rejectionReason("FDC acknowledgement must match R/RGR FDC n/nnnn initials")
                    .build();
        }
        String initials = matcher.group(2).toUpperCase();
        if (!initials.matches("[A-Z]+")) {
            return FdcAcknowledgementCommand.builder()
                    .accepted(false)
                    .notamId(matcher.group(1))
                    .initials(initials)
                    .rejectionReason("FDC ACK initials must be alphabetic")
                    .build();
        }
        return FdcAcknowledgementCommand.builder()
                .accepted(true)
                .notamId(matcher.group(1))
                .initials(initials)
                .build();
    }
}
