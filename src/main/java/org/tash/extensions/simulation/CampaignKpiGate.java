package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class CampaignKpiGate {
    private String metric;
    private String operator;
    private double threshold;
    private boolean blocking;
    private String rationale;
}
