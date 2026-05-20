package org.tash.extensions.messaging;

public enum MessageNetwork {
    AUTODIN('O'),
    AWN('M'),
    CNS('C'),
    NADIN('N'),
    USNOF('D'),
    WMSCR('E'),
    DINS('R'),
    UNKNOWN('?');

    private final char code;

    MessageNetwork(char code) {
        this.code = code;
    }

    public char getCode() {
        return code;
    }

    public static MessageNetwork fromCode(char code) {
        for (MessageNetwork network : values()) {
            if (network.code == code) {
                return network;
            }
        }
        return UNKNOWN;
    }
}
