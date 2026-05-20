package org.tash.extensions.notam;

import lombok.Builder;
import lombok.Data;

import java.util.List;

@Data
@Builder
public class DomesticNotamParseResult {
    private boolean accepted;
    private DomesticNotamRecord record;
    private String rejectionReason;
    private String inferredKeyword;
    private String contractionClassification;
    private String q23;
    private String q45;
    private String qCodeReason;
    private List<String> recognizedContractions;
    private List<String> warnings;
}
