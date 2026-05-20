package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;

import java.util.List;

@Data
@Builder
public class AltrvRouteGraphValidation {
    private boolean valid;
    private List<String> diagnostics;
    private List<AltrvDiagnostic> typedDiagnostics;
}
