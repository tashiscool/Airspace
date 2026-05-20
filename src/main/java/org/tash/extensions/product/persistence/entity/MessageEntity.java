package org.tash.extensions.product.persistence.entity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.Table;

@Entity
@Table(name = "ops_message")
public class MessageEntity extends BaseOperationalEntity {
    @Column(name = "mission_id")
    private java.util.UUID missionId;
    @Column(name = "reservation_id")
    private java.util.UUID reservationId;
    @Column(name = "family", nullable = false)
    private String family;
    @Column(name = "direction", nullable = false)
    private String direction;
    @Column(name = "status", nullable = false)
    private String status;
    @Column(name = "subject")
    private String subject;
    @Column(name = "raw_text", columnDefinition = "TEXT")
    private String rawText;
    @Column(name = "parsed_summary_json", columnDefinition = "TEXT")
    private String parsedSummaryJson;

    public java.util.UUID getMissionId() { return missionId; }
    public void setMissionId(java.util.UUID missionId) { this.missionId = missionId; }
    public java.util.UUID getReservationId() { return reservationId; }
    public void setReservationId(java.util.UUID reservationId) { this.reservationId = reservationId; }
    public String getFamily() { return family; }
    public void setFamily(String family) { this.family = family; }
    public String getDirection() { return direction; }
    public void setDirection(String direction) { this.direction = direction; }
    public String getStatus() { return status; }
    public void setStatus(String status) { this.status = status; }
    public String getSubject() { return subject; }
    public void setSubject(String subject) { this.subject = subject; }
    public String getRawText() { return rawText; }
    public void setRawText(String rawText) { this.rawText = rawText; }
    public String getParsedSummaryJson() { return parsedSummaryJson; }
    public void setParsedSummaryJson(String parsedSummaryJson) { this.parsedSummaryJson = parsedSummaryJson; }
}
