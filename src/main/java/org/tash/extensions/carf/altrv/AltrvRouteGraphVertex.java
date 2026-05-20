package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class AltrvRouteGraphVertex {
    private String id;
    private AltrvRouteGraphVertexType type;
    private String routeId;
    private String eventId;
    private String label;
    private AltrvSourceSpan sourceSpan;
}
