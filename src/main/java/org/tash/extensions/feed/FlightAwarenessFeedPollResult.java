package org.tash.extensions.feed;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.simulation.TrafficReplayBundle;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@Data
@Builder
public class FlightAwarenessFeedPollResult {
    private boolean accepted;
    private boolean setupRequired;
    private boolean liveCallAttempted;
    private String sourceId;
    private String sourceMode;
    private String providerReceiptId;
    private ZonedDateTime receivedAt;
    private TrafficReplayBundle trafficReplay;
    @Builder.Default
    private List<String> providerTransports = new ArrayList<>();
    @Builder.Default
    private List<String> trackingSourceTypes = new ArrayList<>();
    @Builder.Default
    private List<String> coverageWarnings = new ArrayList<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
