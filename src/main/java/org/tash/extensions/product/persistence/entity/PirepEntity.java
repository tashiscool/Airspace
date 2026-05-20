package org.tash.extensions.product.persistence.entity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.Id;
import jakarta.persistence.Table;

import java.time.ZonedDateTime;

@Entity
@Table(name = "ops_pirep")
public class PirepEntity {
    @Id
    private String id;
    @Column(name = "aircraft_id")
    private String aircraftId;
    @Column(name = "aircraft_type")
    private String aircraftType;
    @Column(name = "phenomenon")
    private String phenomenon;
    @Column(name = "intensity")
    private String intensity;
    @Column(name = "observation_time")
    private ZonedDateTime observationTime;
    @Column(name = "latitude")
    private Double latitude;
    @Column(name = "longitude")
    private Double longitude;
    @Column(name = "altitude_feet")
    private Double altitudeFeet;
    @Column(name = "raw_text", columnDefinition = "TEXT")
    private String rawText;
    @Column(name = "ingest_json", nullable = false, columnDefinition = "TEXT")
    private String ingestJson;
    @Column(name = "received_at", nullable = false)
    private ZonedDateTime receivedAt;

    public String getId() { return id; }
    public void setId(String id) { this.id = id; }
    public String getAircraftId() { return aircraftId; }
    public void setAircraftId(String aircraftId) { this.aircraftId = aircraftId; }
    public String getAircraftType() { return aircraftType; }
    public void setAircraftType(String aircraftType) { this.aircraftType = aircraftType; }
    public String getPhenomenon() { return phenomenon; }
    public void setPhenomenon(String phenomenon) { this.phenomenon = phenomenon; }
    public String getIntensity() { return intensity; }
    public void setIntensity(String intensity) { this.intensity = intensity; }
    public ZonedDateTime getObservationTime() { return observationTime; }
    public void setObservationTime(ZonedDateTime observationTime) { this.observationTime = observationTime; }
    public Double getLatitude() { return latitude; }
    public void setLatitude(Double latitude) { this.latitude = latitude; }
    public Double getLongitude() { return longitude; }
    public void setLongitude(Double longitude) { this.longitude = longitude; }
    public Double getAltitudeFeet() { return altitudeFeet; }
    public void setAltitudeFeet(Double altitudeFeet) { this.altitudeFeet = altitudeFeet; }
    public String getRawText() { return rawText; }
    public void setRawText(String rawText) { this.rawText = rawText; }
    public String getIngestJson() { return ingestJson; }
    public void setIngestJson(String ingestJson) { this.ingestJson = ingestJson; }
    public ZonedDateTime getReceivedAt() { return receivedAt; }
    public void setReceivedAt(ZonedDateTime receivedAt) { this.receivedAt = receivedAt; }
}
