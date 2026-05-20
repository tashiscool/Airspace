package org.tash.extensions.messaging;

import lombok.Builder;
import lombok.Data;

import java.util.List;

@Data
@Builder
public class UsnsRoutingOutcome {
    private UsnsRoutingOutcomeType type;
    private MrsFunctionCode functionCode;
    private boolean accepted;
    private boolean returnToSender;
    private boolean bypassErq;
    private boolean privilegedRequest;
    private List<String> warnings;
    private List<String> errors;
}
