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
    private String semanticFacilityFamily;
    private String semanticCondition;
    private String semanticAction;
    private String reducerRuleId;
    private String reducerName;
    private DomesticNotamSemanticClassification semanticClassification;
    private List<String> recognizedContractions;
    private List<String> warnings;
}
