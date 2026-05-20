package org.tash.extensions.messaging;

import lombok.Builder;
import lombok.Data;

import java.util.List;

@Data
@Builder
public class UsnsMessageParseResult {
    private boolean accepted;
    private UsnsMessageEnvelope envelope;
    private List<String> warnings;
    private List<String> errors;
    private UsnsRoutingDecision routingDecision;
    private String normalizedBody;
}
