package org.tash.extensions.feed;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.carf.api.CarfAnalysisResult;
import org.tash.extensions.messaging.UsnsIngestResult;
import org.tash.extensions.notam.NotamAirspaceRestriction;
import org.tash.extensions.weather.pirep.PirepIngestResult;
import org.tash.extensions.weather.product.WeatherProductParseResult;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class OperationalFeedIngestResult {
    private final OperationalFeedEnvelope envelope;
    private final boolean accepted;
    private final String rawPayloadHash;
    private final UsnsIngestResult usnsResult;
    private final CarfAnalysisResult carfResult;
    private final WeatherProductParseResult weatherProductResult;
    private final PirepIngestResult pirepResult;
    private final NotamAirspaceRestriction notamRestriction;
    @Builder.Default
    private final List<String> downstreamArtifactIds = new ArrayList<>();
    @Builder.Default
    private final List<String> warnings = new ArrayList<>();
    @Builder.Default
    private final List<String> errors = new ArrayList<>();

    public List<String> getDownstreamArtifactIds() {
        return Collections.unmodifiableList(downstreamArtifactIds == null ? Collections.emptyList() : downstreamArtifactIds);
    }

    public List<String> getWarnings() {
        return Collections.unmodifiableList(warnings == null ? Collections.emptyList() : warnings);
    }

    public List<String> getErrors() {
        return Collections.unmodifiableList(errors == null ? Collections.emptyList() : errors);
    }
}
