package org.tash.extensions.workflow;

public enum ReservationWorkflowState {
    DRAFT,
    VALIDATED,
    SUBMITTED,
    APPROVED,
    REJECTED,
    CANCELLED,
    LOCKED,
    STALE_LOCK,
    COMPLETED
}
