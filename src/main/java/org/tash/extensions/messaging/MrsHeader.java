package org.tash.extensions.messaging;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class MrsHeader {
    private String raw;
    private String functionCode;
    private String priority;
    private String sourceNetworkCode;
    private MessageNetwork sourceNetwork;
    private String messageFormatIndicator;
    private String destination;
    private String terminalId;
    private boolean valid;
    private String error;

    public static MrsHeader parse(String raw) {
        String value = raw == null ? "" : raw;
        if (value.length() < 10) {
            return MrsHeader.builder()
                    .raw(value)
                    .valid(false)
                    .error("MRS header length < 10")
                    .sourceNetwork(MessageNetwork.UNKNOWN)
                    .build();
        }
        String firstTen = value.substring(0, 10);
        char networkCode = firstTen.charAt(4);
        return MrsHeader.builder()
                .raw(firstTen)
                .functionCode(firstTen.substring(0, 2))
                .priority(firstTen.substring(2, 4))
                .sourceNetworkCode(firstTen.substring(4, 5))
                .sourceNetwork(MessageNetwork.fromCode(networkCode))
                .messageFormatIndicator(firstTen.substring(5, 6))
                .destination(firstTen.substring(6, 8))
                .terminalId(firstTen.substring(8, 10))
                .valid(true)
                .build();
    }
}
