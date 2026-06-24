package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class SectorCapacityModel {
    private String capacityId;
    private String sectorId;
    private int baselineCapacity;
    private int reducedCapacity;
    private int demandCount;
    private int startOffsetMinutes;
    private int endOffsetMinutes;
    private String capacityState;
    private String reason;
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
