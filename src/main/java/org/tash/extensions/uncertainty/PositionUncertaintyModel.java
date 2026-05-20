package org.tash.extensions.uncertainty;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class PositionUncertaintyModel {
    private final double horizontalNauticalMiles;
    private final double verticalFeet;
    @Builder.Default
    private final double confidence = 1.0;
}
