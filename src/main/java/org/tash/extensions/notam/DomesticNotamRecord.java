package org.tash.extensions.notam;

import lombok.Builder;
import lombok.Data;

import java.time.ZonedDateTime;

/**
 * Parsed domestic NOTAM fields matching the legacy DOM1 Visual Parse++ grammar.
 */
@Data
@Builder
public class DomesticNotamRecord {
    public enum Type {
        NEW,
        EDIT,
        CANCEL
    }

    private Type type;
    private String accountability;
    private String notamNumber;
    private String cancelNumber;
    private String location;
    private String keyword;
    private boolean unofficial;
    private String text;
    private ZonedDateTime effectiveStart;
    private ZonedDateTime effectiveEnd;
    private boolean estimatedEnd;
    private String comment;
}
