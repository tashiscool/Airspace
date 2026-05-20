package org.tash.extensions.workflow;

import java.util.List;
import java.util.Optional;

public interface ReservationWorkflowRepository {
    ReservationWorkflowRecord save(ReservationWorkflowRecord record);
    Optional<ReservationWorkflowRecord> findById(String id);
    List<ReservationWorkflowRecord> findOpen();
    ReservationWorkflowRecord appendAuditEvent(String id, ReservationWorkflowAuditEvent event);
}
