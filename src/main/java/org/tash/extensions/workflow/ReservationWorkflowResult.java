package org.tash.extensions.workflow;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class ReservationWorkflowResult {
    private boolean accepted;
    private ReservationWorkflowRecord record;
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
