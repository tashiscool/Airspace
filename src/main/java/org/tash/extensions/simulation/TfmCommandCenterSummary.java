package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class TfmCommandCenterSummary {
    private String id;
    private String generatedAt;
    private String boardMode;
    private String sourceMode;
    private String authorizationMode;
    private int selectedMinute;
    private NationalDemandCapacityReport demandCapacityReport;
    private NationalDemandCapacitySnapshot selectedSnapshot;
    private TfmImpactTotals impactTotals;
    @Builder.Default
    private List<TfmAirportDemandSummary> airportDemand = new ArrayList<>();
    @Builder.Default
    private List<TfmSectorLoadSummary> sectorLoad = new ArrayList<>();
    @Builder.Default
    private List<TfmActiveConstraintSummary> activeConstraints = new ArrayList<>();
    @Builder.Default
    private List<TfmProposedTmiSummary> proposedTmis = new ArrayList<>();
    @Builder.Default
    private List<TfmRouteAlternativeSummary> routeAlternatives = new ArrayList<>();
    @Builder.Default
    private List<String> humanFactorsNotes = new ArrayList<>();
    @Builder.Default
    private List<String> assumptions = new ArrayList<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
