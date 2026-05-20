package org.tash.extensions.messaging;

import lombok.Builder;
import lombok.Data;

import java.util.List;

@Data
@Builder
public class UsnsMessageEnvelope {
    private String rawText;
    private MrsHeader mrsHeader;
    private String heading;
    private String addressLine;
    private String originLine;
    private String originDhm;
    private String originAddress;
    private String priority;
    private List<String> toAddresses;
    private String body;
    private boolean wmscr;
    private boolean usnof;
}
