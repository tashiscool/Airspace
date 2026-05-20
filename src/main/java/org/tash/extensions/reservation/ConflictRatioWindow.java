package org.tash.extensions.reservation;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class ConflictRatioWindow {
    private double rawStartRatio;
    private double rawEndRatio;
    private double factoredStartRatio;
    private double factoredEndRatio;
}
