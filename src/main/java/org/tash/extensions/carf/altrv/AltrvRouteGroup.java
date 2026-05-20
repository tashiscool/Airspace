package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;

import java.util.List;

@Data
@Builder
public class AltrvRouteGroup {
    private String id;
    private List<AltrvRoute> routes;
}
