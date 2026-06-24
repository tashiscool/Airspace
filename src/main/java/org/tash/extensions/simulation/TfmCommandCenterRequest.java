package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class TfmCommandCenterRequest {
    private NationalDemandCapacityConfig demandCapacityConfig;
    private Integer focusMinute;
    @Builder.Default
    private int maxAirportRows = 12;
    @Builder.Default
    private int maxSectorRows = 12;
    @Builder.Default
    private int maxConstraintRows = 20;
    @Builder.Default
    private int maxProposalRows = 20;
    @Builder.Default
    private int maxRouteAlternatives = 12;
}
