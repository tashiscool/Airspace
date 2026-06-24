package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class OutcomeMetricSummary {
    private String id;
    private String label;
    private double value;
    private String unit;
    private String status;
    private String rationale;
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
