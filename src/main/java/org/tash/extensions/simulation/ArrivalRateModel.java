package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class ArrivalRateModel {
    private String rateId;
    private String airportId;
    private String runwayConfiguration;
    private int acceptanceRatePerHour;
    private int demandRatePerHour;
    private int startOffsetMinutes;
    private int endOffsetMinutes;
    private String reason;
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
