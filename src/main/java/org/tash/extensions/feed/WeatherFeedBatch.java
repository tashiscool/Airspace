package org.tash.extensions.feed;

import lombok.Builder;
import lombok.Data;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@Data
@Builder
public class WeatherFeedBatch {
    private boolean accepted;
    private String sourceId;
    private ZonedDateTime receivedAt;
    private OperationalFeedPollResult pollResult;
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
