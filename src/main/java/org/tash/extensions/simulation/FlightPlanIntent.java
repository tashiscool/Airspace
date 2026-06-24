package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class FlightPlanIntent {
    private String origin;
    private String destination;
    private String requestedAltitudeBlock;
    @Builder.Default
    private List<List<Double>> route = new ArrayList<>();
    private String missionId;
    private String reservationId;
    private String rationale;
}
