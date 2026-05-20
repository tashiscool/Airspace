package org.tash.extensions.carf.lifecycle;

import lombok.Builder;
import lombok.Data;

import java.util.List;

@Data
@Builder
public class ReservationLifecycleResult {
    private ReservationLifecycleState state;
    private List<String> actions;
    private List<String> diagnostics;
}
