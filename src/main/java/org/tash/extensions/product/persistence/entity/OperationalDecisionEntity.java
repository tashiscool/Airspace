package org.tash.extensions.product.persistence.entity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.Table;

@Entity
@Table(name = "ops_decision")
public class OperationalDecisionEntity extends BaseOperationalEntity {
    @Column(name = "action", nullable = false)
    private String action;
    @Column(name = "recommended_action")
    private String recommendedAction;
    @Column(name = "confidence", nullable = false)
    private double confidence;
    @Column(name = "rationale", columnDefinition = "TEXT")
    private String rationale;
    @Column(name = "result_json", nullable = false, columnDefinition = "TEXT")
    private String resultJson;
    @Column(name = "audit_json", columnDefinition = "TEXT")
    private String auditJson;
    @Column(name = "replay_json", columnDefinition = "TEXT")
    private String replayJson;

    public String getAction() { return action; }
    public void setAction(String action) { this.action = action; }
    public String getRecommendedAction() { return recommendedAction; }
    public void setRecommendedAction(String recommendedAction) { this.recommendedAction = recommendedAction; }
    public double getConfidence() { return confidence; }
    public void setConfidence(double confidence) { this.confidence = confidence; }
    public String getRationale() { return rationale; }
    public void setRationale(String rationale) { this.rationale = rationale; }
    public String getResultJson() { return resultJson; }
    public void setResultJson(String resultJson) { this.resultJson = resultJson; }
    public String getAuditJson() { return auditJson; }
    public void setAuditJson(String auditJson) { this.auditJson = auditJson; }
    public String getReplayJson() { return replayJson; }
    public void setReplayJson(String replayJson) { this.replayJson = replayJson; }
}
