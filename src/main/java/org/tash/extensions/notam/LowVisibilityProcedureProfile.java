package org.tash.extensions.notam;

import lombok.Builder;
import lombok.Data;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Local reference-data shape for reconciling FAA/ICAO/operator low-visibility
 * terminology. This does not declare live airport procedure state.
 */
@Data
@Builder
public class LowVisibilityProcedureProfile {
    private final String airportId;
    private final String localProcedureName;
    private final String faaTerminology;
    private final String icaoTerminology;
    private final Integer advisoryRvrThresholdFeet;
    private final boolean smgcsAvailable;
    private final boolean lvpEquivalentAvailable;
    private final String source;
    private final String sourceVersion;
    private final ZonedDateTime effectiveAt;
    @Builder.Default
    private final List<String> rvrComponents = new ArrayList<>();
    @Builder.Default
    private final List<String> reviewAuthorities = new ArrayList<>();

    public List<String> getRvrComponents() {
        return Collections.unmodifiableList(rvrComponents == null ? Collections.emptyList() : rvrComponents);
    }

    public List<String> getReviewAuthorities() {
        return Collections.unmodifiableList(reviewAuthorities == null ? Collections.emptyList() : reviewAuthorities);
    }
}
