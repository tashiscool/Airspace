package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;

import java.util.List;

@Data
@Builder
public class AltrvDestination {
    private String rawText;
    private String fixId;
    private List<AltrvCallsign> callsigns;
    private AltrvSourceSpan sourceSpan;
}
