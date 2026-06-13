package org.tash.extensions.agentic;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AgenticRiskProfile {
    private String autonomyScope;
    private String toolSurface;
    private String worstCaseBlastRadius;
    private String permissionScope;
    private String dataEgress;
    private String poisonedDataExposure;
    private String auditAttribution;
    private String toolProvenance;
    private String modelProvenance;
    private String rollbackPath;
    private String costClass;
    private String slaClass;
    private HumanReviewMode requiredHumanReviewMode;
}
