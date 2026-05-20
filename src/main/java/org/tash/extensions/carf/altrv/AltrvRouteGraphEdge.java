package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;

import java.util.List;

@Data
@Builder
public class AltrvRouteGraphEdge {
    private String sourceId;
    private String targetId;
    private List<AltrvCallsign> callsigns;
    private String reason;
}
