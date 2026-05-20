package org.tash.extensions.notam;

import lombok.Builder;
import lombok.Data;

import java.time.ZonedDateTime;
import java.util.Set;

@Data
@Builder
public class DomesticSurfaceCondition {
    public enum Hazard {
        BERM,
        SNOW_PILE,
        TURNAROUND,
        SNOW,
        ICE,
        SLUSH,
        FROST,
        WATER,
        DEICED_LIQUID
    }

    private DomesticNotamRecord.Type type;
    private String accountability;
    private String notamNumber;
    private String location;
    private String keyword;
    private String affectedSurface;
    private Set<Hazard> hazards;
    private Double bermHeightInches;
    private Double snowPileHeightFeet;
    private ZonedDateTime effectiveStart;
    private ZonedDateTime effectiveEnd;
    private String description;
}
