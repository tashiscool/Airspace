package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class AltrvAircraftType {
    private int count;
    private String type;
}
