package org.tash.extensions.product.persistence.entity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.Table;

import java.time.ZonedDateTime;
import java.util.UUID;

@Entity
@Table(name = "ops_notam")
public class NotamEntity extends BaseOperationalEntity {
    @Column(name = "reservation_id")
    private UUID reservationId;
    @Column(name = "notam_type")
    private String notamType;
    @Column(name = "q_code")
    private String qCode;
    @Column(name = "affected_location")
    private String affectedLocation;
    @Column(name = "lower_altitude_feet")
    private Double lowerAltitudeFeet;
    @Column(name = "upper_altitude_feet")
    private Double upperAltitudeFeet;
    @Column(name = "effective_start")
    private ZonedDateTime effectiveStart;
    @Column(name = "effective_end")
    private ZonedDateTime effectiveEnd;
    @Column(name = "raw_text", columnDefinition = "TEXT")
    private String rawText;
    @Column(name = "parsed_json", columnDefinition = "TEXT")
    private String parsedJson;

    public UUID getReservationId() { return reservationId; }
    public void setReservationId(UUID reservationId) { this.reservationId = reservationId; }
    public String getNotamType() { return notamType; }
    public void setNotamType(String notamType) { this.notamType = notamType; }
    public String getQCode() { return qCode; }
    public void setQCode(String qCode) { this.qCode = qCode; }
    public String getAffectedLocation() { return affectedLocation; }
    public void setAffectedLocation(String affectedLocation) { this.affectedLocation = affectedLocation; }
    public Double getLowerAltitudeFeet() { return lowerAltitudeFeet; }
    public void setLowerAltitudeFeet(Double lowerAltitudeFeet) { this.lowerAltitudeFeet = lowerAltitudeFeet; }
    public Double getUpperAltitudeFeet() { return upperAltitudeFeet; }
    public void setUpperAltitudeFeet(Double upperAltitudeFeet) { this.upperAltitudeFeet = upperAltitudeFeet; }
    public ZonedDateTime getEffectiveStart() { return effectiveStart; }
    public void setEffectiveStart(ZonedDateTime effectiveStart) { this.effectiveStart = effectiveStart; }
    public ZonedDateTime getEffectiveEnd() { return effectiveEnd; }
    public void setEffectiveEnd(ZonedDateTime effectiveEnd) { this.effectiveEnd = effectiveEnd; }
    public String getRawText() { return rawText; }
    public void setRawText(String rawText) { this.rawText = rawText; }
    public String getParsedJson() { return parsedJson; }
    public void setParsedJson(String parsedJson) { this.parsedJson = parsedJson; }
}
