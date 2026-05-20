package org.tash.extensions.messaging;

import java.util.Locale;

public enum MrsFunctionCode {
    NOTAM_TRAFFIC("01"),
    UNCANCEL_NOTAM("05"),
    EDIT_NOTAM("07"),
    DOMESTIC_CROSSOVER("08"),
    INTERNATIONAL_CROSSOVER("09"),
    FDC_COMMENTS_EDIT("17"),
    TIMER_TRAFFIC("22"),
    PRIVILEGED_REQUEST("30"),
    UNKNOWN("");

    private final String code;

    MrsFunctionCode(String code) {
        this.code = code;
    }

    public String getCode() {
        return code;
    }

    public static MrsFunctionCode from(String code) {
        String normalized = code == null ? "" : code.trim().toUpperCase(Locale.US);
        for (MrsFunctionCode value : values()) {
            if (value.code.equals(normalized)) {
                return value;
            }
        }
        return UNKNOWN;
    }
}
