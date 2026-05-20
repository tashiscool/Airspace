package org.tash.extensions.messaging;

public enum UsnsRoutingOutcomeType {
    ACCEPTED,
    REJECTED,
    PARSE_FAILURE,
    RETURN_TO_SENDER,
    BYPASS_ERQ,
    ADMIN_MESSAGE,
    CLASSIFIED_ONLY
}
