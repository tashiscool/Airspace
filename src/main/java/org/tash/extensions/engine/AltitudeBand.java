package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class AltitudeBand {
    private final double lowerFeet;
    private final double upperFeet;
}
