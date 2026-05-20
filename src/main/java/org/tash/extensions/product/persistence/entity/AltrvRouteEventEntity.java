package org.tash.extensions.product.persistence.entity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.FetchType;
import jakarta.persistence.JoinColumn;
import jakarta.persistence.ManyToOne;
import jakarta.persistence.Table;

@Entity
@Table(name = "ops_altrv_route_event")
public class AltrvRouteEventEntity extends BaseOperationalEntity {
    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "route_id")
    private AltrvRouteEntity route;
    @Column(name = "event_type", nullable = false)
    private String eventType;
    @Column(name = "callsign")
    private String callsign;
    @Column(name = "lower_altitude_feet")
    private Double lowerAltitudeFeet;
    @Column(name = "upper_altitude_feet")
    private Double upperAltitudeFeet;
    @Column(name = "duration_seconds")
    private Integer durationSeconds;
    @Column(name = "source_text", columnDefinition = "TEXT")
    private String sourceText;
    @Column(name = "metadata_json", columnDefinition = "TEXT")
    private String metadataJson;
    @Column(name = "sequence_index", nullable = false)
    private int sequenceIndex;

    public AltrvRouteEntity getRoute() { return route; }
    public void setRoute(AltrvRouteEntity route) { this.route = route; }
    public String getEventType() { return eventType; }
    public void setEventType(String eventType) { this.eventType = eventType; }
    public String getCallsign() { return callsign; }
    public void setCallsign(String callsign) { this.callsign = callsign; }
    public Double getLowerAltitudeFeet() { return lowerAltitudeFeet; }
    public void setLowerAltitudeFeet(Double lowerAltitudeFeet) { this.lowerAltitudeFeet = lowerAltitudeFeet; }
    public Double getUpperAltitudeFeet() { return upperAltitudeFeet; }
    public void setUpperAltitudeFeet(Double upperAltitudeFeet) { this.upperAltitudeFeet = upperAltitudeFeet; }
    public Integer getDurationSeconds() { return durationSeconds; }
    public void setDurationSeconds(Integer durationSeconds) { this.durationSeconds = durationSeconds; }
    public String getSourceText() { return sourceText; }
    public void setSourceText(String sourceText) { this.sourceText = sourceText; }
    public String getMetadataJson() { return metadataJson; }
    public void setMetadataJson(String metadataJson) { this.metadataJson = metadataJson; }
    public int getSequenceIndex() { return sequenceIndex; }
    public void setSequenceIndex(int sequenceIndex) { this.sequenceIndex = sequenceIndex; }
}
