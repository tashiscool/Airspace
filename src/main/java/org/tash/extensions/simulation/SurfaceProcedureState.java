package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class SurfaceProcedureState {
    private String procedureFamily;
    private boolean active;
    private boolean terminologyAmbiguity;
    private String localProcedureName;
    private String requestedPhraseology;
    private String confirmationStatus;
    private String rationale;
}
