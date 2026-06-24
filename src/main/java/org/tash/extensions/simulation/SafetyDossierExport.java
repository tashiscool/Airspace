package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class SafetyDossierExport {
    private String id;
    private String campaignId;
    private ZonedDateTime generatedAt;
    private String format;
    private String markdown;
    @Builder.Default
    private Map<String, Object> jsonSummary = new java.util.LinkedHashMap<>();
    @Builder.Default
    private List<String> scenarios = new ArrayList<>();
    @Builder.Default
    private List<String> assumptions = new ArrayList<>();
    @Builder.Default
    private List<String> knownGaps = new ArrayList<>();
    @Builder.Default
    private List<String> nonCertificationWarnings = new ArrayList<>();
    @Builder.Default
    private List<String> replayHashes = new ArrayList<>();
}
