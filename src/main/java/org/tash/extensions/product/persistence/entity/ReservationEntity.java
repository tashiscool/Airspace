package org.tash.extensions.product.persistence.entity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.FetchType;
import jakarta.persistence.JoinColumn;
import jakarta.persistence.ManyToOne;
import jakarta.persistence.Table;

import java.time.ZonedDateTime;

@Entity
@Table(name = "ops_reservation")
public class ReservationEntity extends BaseOperationalEntity {
    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "mission_id")
    private MissionEntity mission;
    @Column(name = "reservation_key")
    private String reservationKey;
    @Column(name = "state", nullable = false)
    private String state = "DRAFT";
    @Column(name = "raw_text", columnDefinition = "TEXT")
    private String rawText;
    @Column(name = "lower_altitude_feet")
    private Double lowerAltitudeFeet;
    @Column(name = "upper_altitude_feet")
    private Double upperAltitudeFeet;
    @Column(name = "effective_start")
    private ZonedDateTime effectiveStart;
    @Column(name = "effective_end")
    private ZonedDateTime effectiveEnd;
    @Column(name = "locked_by")
    private String lockedBy;
    @Column(name = "locked_at")
    private ZonedDateTime lockedAt;
    @Column(name = "last_analysis_json", columnDefinition = "TEXT")
    private String lastAnalysisJson;

    public MissionEntity getMission() { return mission; }
    public void setMission(MissionEntity mission) { this.mission = mission; }
    public String getReservationKey() { return reservationKey; }
    public void setReservationKey(String reservationKey) { this.reservationKey = reservationKey; }
    public String getState() { return state; }
    public void setState(String state) { this.state = state; }
    public String getRawText() { return rawText; }
    public void setRawText(String rawText) { this.rawText = rawText; }
    public Double getLowerAltitudeFeet() { return lowerAltitudeFeet; }
    public void setLowerAltitudeFeet(Double lowerAltitudeFeet) { this.lowerAltitudeFeet = lowerAltitudeFeet; }
    public Double getUpperAltitudeFeet() { return upperAltitudeFeet; }
    public void setUpperAltitudeFeet(Double upperAltitudeFeet) { this.upperAltitudeFeet = upperAltitudeFeet; }
    public ZonedDateTime getEffectiveStart() { return effectiveStart; }
    public void setEffectiveStart(ZonedDateTime effectiveStart) { this.effectiveStart = effectiveStart; }
    public ZonedDateTime getEffectiveEnd() { return effectiveEnd; }
    public void setEffectiveEnd(ZonedDateTime effectiveEnd) { this.effectiveEnd = effectiveEnd; }
    public String getLockedBy() { return lockedBy; }
    public void setLockedBy(String lockedBy) { this.lockedBy = lockedBy; }
    public ZonedDateTime getLockedAt() { return lockedAt; }
    public void setLockedAt(ZonedDateTime lockedAt) { this.lockedAt = lockedAt; }
    public String getLastAnalysisJson() { return lastAnalysisJson; }
    public void setLastAnalysisJson(String lastAnalysisJson) { this.lastAnalysisJson = lastAnalysisJson; }
}
