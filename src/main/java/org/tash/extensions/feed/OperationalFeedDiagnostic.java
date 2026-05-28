package org.tash.extensions.feed;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class OperationalFeedDiagnostic {
    private final OperationalFeedAuthorityMode authorityMode;
    private final String code;
    private final String sourceId;
    private final String message;
}
