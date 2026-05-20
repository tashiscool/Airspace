package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class GeometryOverlapResult {
    private final boolean overlaps;
    private final GeometryOverlapType type;
    private final double minimumDistanceNauticalMiles;
    private final double confidence;
    @Builder.Default
    private final List<Integer> leftSegmentIndexes = new ArrayList<>();
    @Builder.Default
    private final List<Integer> rightSegmentIndexes = new ArrayList<>();

    public List<Integer> getLeftSegmentIndexes() {
        return Collections.unmodifiableList(leftSegmentIndexes == null ? Collections.emptyList() : leftSegmentIndexes);
    }

    public List<Integer> getRightSegmentIndexes() {
        return Collections.unmodifiableList(rightSegmentIndexes == null ? Collections.emptyList() : rightSegmentIndexes);
    }
}
