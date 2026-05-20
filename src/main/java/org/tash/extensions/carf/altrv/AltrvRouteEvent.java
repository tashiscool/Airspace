package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;

import java.util.Map;

@Data
@Builder
public class AltrvRouteEvent {
    private AltrvRouteEventType type;
    private String sourceText;
    private String eventId;
    private AltrvSourceSpan sourceSpan;
    private Map<String, String> metadata;
}
