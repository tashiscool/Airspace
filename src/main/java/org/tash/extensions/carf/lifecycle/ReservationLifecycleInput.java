package org.tash.extensions.carf.lifecycle;

import lombok.Builder;
import lombok.Data;

import java.time.Duration;
import java.time.ZonedDateTime;

@Data
@Builder
public class ReservationLifecycleInput {
    private ReservationLifecycleState currentState;
    private ZonedDateTime reservationEndTime;
    private ZonedDateTime lockedAt;
    private ZonedDateTime evaluationTime;
    private Duration completionBuffer;
    private Duration staleLockThreshold;
}
