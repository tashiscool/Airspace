package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class AirportOpsTimeline {
    private String airportId;
    private String sourceMode;
    @Builder.Default
    private List<RunwayOpsState> runwayStates = new ArrayList<>();
    @Builder.Default
    private List<SurfaceProcedureState> surfaceProcedures = new ArrayList<>();
    private String assumptions;
}
