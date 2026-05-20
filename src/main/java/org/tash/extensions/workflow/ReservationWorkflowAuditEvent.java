package org.tash.extensions.workflow;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.time.ZonedDateTime;

@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class ReservationWorkflowAuditEvent {
    private ReservationWorkflowCommand command;
    private String actor;
    private ZonedDateTime timestamp;
    private String note;
}
