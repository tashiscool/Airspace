package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class AltrvCallsign {
    private String name;
    private String number;

    public String compact() {
        return (name == null ? "" : name) + (number == null ? "" : number);
    }
}
