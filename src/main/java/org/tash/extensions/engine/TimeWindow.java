package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;

import java.time.ZonedDateTime;

@Data
@Builder
public class TimeWindow {
    private final ZonedDateTime start;
    private final ZonedDateTime end;
}
