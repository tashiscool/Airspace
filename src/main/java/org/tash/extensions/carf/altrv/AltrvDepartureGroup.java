package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;

import java.util.List;

@Data
@Builder
public class AltrvDepartureGroup {
    private String rawText;
    private List<AltrvCallsign> callsigns;
}
