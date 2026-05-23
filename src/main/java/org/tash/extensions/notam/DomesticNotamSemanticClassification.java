package org.tash.extensions.notam;

import lombok.Builder;
import lombok.Data;

import java.util.List;

@Data
@Builder
public class DomesticNotamSemanticClassification {
    private String facilityFamily;
    private String condition;
    private String action;
    private String q23;
    private String q45;
    private String reducerRuleId;
    private String reducerName;
    private List<String> recognizedContractions;
    private List<String> warnings;
}
