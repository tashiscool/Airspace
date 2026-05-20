package org.tash.extensions.carf.lifecycle;

public enum ReservationLifecycleState {
    DRAFT,
    SUBMITTED,
    APPROVED,
    FLOWN,
    COMPLETED,
    LOCKED,
    STALE_LOCK
}
