package org.tash.extensions.carf.lifecycle;

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ReservationLifecycleService {
    public ReservationLifecycleResult evaluate(ReservationLifecycleInput input) {
        List<String> actions = new ArrayList<>();
        List<String> diagnostics = new ArrayList<>();
        if (input == null) {
            diagnostics.add("No lifecycle input supplied");
            return result(ReservationLifecycleState.DRAFT, actions, diagnostics);
        }
        ReservationLifecycleState state = input.getCurrentState() == null
                ? ReservationLifecycleState.DRAFT
                : input.getCurrentState();
        ZonedDateTime now = input.getEvaluationTime();
        if (now == null) {
            diagnostics.add("No evaluation time supplied");
            return result(state, actions, diagnostics);
        }

        Duration completionBuffer = input.getCompletionBuffer() == null
                ? Duration.ZERO
                : input.getCompletionBuffer();
        if (state == ReservationLifecycleState.APPROVED && input.getReservationEndTime() != null
                && !now.isBefore(input.getReservationEndTime().plus(completionBuffer))) {
            state = ReservationLifecycleState.COMPLETED;
            actions.add("mark-approved-reservation-complete-after-flown");
        }

        Duration staleLockThreshold = input.getStaleLockThreshold() == null
                ? Duration.ofMinutes(30)
                : input.getStaleLockThreshold();
        if (input.getLockedAt() != null && !now.isBefore(input.getLockedAt().plus(staleLockThreshold))) {
            if (state == ReservationLifecycleState.LOCKED) {
                state = ReservationLifecycleState.STALE_LOCK;
            }
            actions.add("release-stale-lock");
        }

        return result(state, actions, diagnostics);
    }

    private ReservationLifecycleResult result(ReservationLifecycleState state,
                                              List<String> actions,
                                              List<String> diagnostics) {
        return ReservationLifecycleResult.builder()
                .state(state)
                .actions(Collections.unmodifiableList(new ArrayList<>(actions)))
                .diagnostics(Collections.unmodifiableList(new ArrayList<>(diagnostics)))
                .build();
    }
}
