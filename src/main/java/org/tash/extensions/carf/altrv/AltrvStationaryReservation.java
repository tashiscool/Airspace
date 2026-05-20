package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;

import java.util.List;

@Data
@Builder
public class AltrvStationaryReservation {
    private String reservationText;
    private String commentsText;
    private List<AltrvArea> areas;
}
