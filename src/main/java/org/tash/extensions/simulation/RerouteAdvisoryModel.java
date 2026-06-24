package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class RerouteAdvisoryModel {
    private String advisoryId;
    private String advisoryType;
    private boolean required;
    private String routeName;
    private String routeText;
    @Builder.Default
    private List<List<Double>> routePoints = new ArrayList<>();
    private String reason;
    private int startOffsetMinutes;
    private int endOffsetMinutes;
    @Builder.Default
    private List<String> affectedFlightIds = new ArrayList<>();
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
