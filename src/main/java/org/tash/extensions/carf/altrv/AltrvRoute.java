package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;

import java.util.List;

@Data
@Builder
public class AltrvRoute {
    private String id;
    private AltrvRouteKind kind;
    private String rawText;
    private List<AltrvRoutePoint> points;
    private List<AltrvRouteEvent> events;
    private List<AltrvRouteState> states;
    private boolean reverse;
    private boolean partial;
    private boolean common;
    private boolean branch;
    private boolean implicit;
}
