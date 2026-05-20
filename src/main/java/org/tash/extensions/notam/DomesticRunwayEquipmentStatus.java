package org.tash.extensions.notam;

import lombok.Builder;
import lombok.Data;

import java.time.ZonedDateTime;
import java.util.List;
import java.util.Set;

@Data
@Builder
public class DomesticRunwayEquipmentStatus {
    public enum Equipment {
        DISTANCE_REMAINING_SIGN,
        DISTANCE_REMAINING_MARKER,
        RUNWAY_LIGHTS,
        RUNWAY_MARKINGS
    }

    public enum Status {
        OUT_OF_SERVICE,
        UNLIGHTED,
        UNLIT,
        OBSCURED,
        MISSING,
        REMOVED,
        NONSTANDARD,
        NONCONFORMING
    }

    private DomesticNotamRecord.Type type;
    private String accountability;
    private String notamNumber;
    private String location;
    private String runway;
    private List<Integer> distancesFeet;
    private Set<Equipment> equipment;
    private Set<Status> statuses;
    private ZonedDateTime effectiveStart;
    private ZonedDateTime effectiveEnd;
    private String description;
}
