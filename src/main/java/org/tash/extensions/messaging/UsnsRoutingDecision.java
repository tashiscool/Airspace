package org.tash.extensions.messaging;

public enum UsnsRoutingDecision {
    ACCEPT_USNS,
    ADMIN_MESSAGE,
    NOT_ADDRESSED_TO_USNS,
    RETURN_TO_SENDER,
    REJECTED,
    NAT_TRACK,
    PRIVILEGED_REQUEST,
    UNKNOWN
}
