package org.tash.extensions.product.persistence.entity;

import jakarta.persistence.CascadeType;
import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.FetchType;
import jakarta.persistence.JoinColumn;
import jakarta.persistence.ManyToOne;
import jakarta.persistence.OneToMany;
import jakarta.persistence.OrderBy;
import jakarta.persistence.Table;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@Entity
@Table(name = "ops_altrv_route")
public class AltrvRouteEntity extends BaseOperationalEntity {
    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "route_group_id", nullable = false)
    private AltrvRouteGroupEntity routeGroup;
    @Column(name = "route_name")
    private String routeName;
    @Column(name = "route_family", nullable = false)
    private String routeFamily;
    @Column(name = "lower_altitude_feet")
    private Double lowerAltitudeFeet;
    @Column(name = "upper_altitude_feet")
    private Double upperAltitudeFeet;
    @Column(name = "effective_start")
    private ZonedDateTime effectiveStart;
    @Column(name = "effective_end")
    private ZonedDateTime effectiveEnd;
    @Column(name = "source_text", columnDefinition = "TEXT")
    private String sourceText;
    @Column(name = "sequence_index", nullable = false)
    private int sequenceIndex;
    @OneToMany(mappedBy = "route", cascade = CascadeType.ALL, orphanRemoval = true)
    @OrderBy("sequenceIndex ASC")
    private List<AltrvRouteEventEntity> events = new ArrayList<>();

    public AltrvRouteGroupEntity getRouteGroup() { return routeGroup; }
    public void setRouteGroup(AltrvRouteGroupEntity routeGroup) { this.routeGroup = routeGroup; }
    public String getRouteName() { return routeName; }
    public void setRouteName(String routeName) { this.routeName = routeName; }
    public String getRouteFamily() { return routeFamily; }
    public void setRouteFamily(String routeFamily) { this.routeFamily = routeFamily; }
    public Double getLowerAltitudeFeet() { return lowerAltitudeFeet; }
    public void setLowerAltitudeFeet(Double lowerAltitudeFeet) { this.lowerAltitudeFeet = lowerAltitudeFeet; }
    public Double getUpperAltitudeFeet() { return upperAltitudeFeet; }
    public void setUpperAltitudeFeet(Double upperAltitudeFeet) { this.upperAltitudeFeet = upperAltitudeFeet; }
    public ZonedDateTime getEffectiveStart() { return effectiveStart; }
    public void setEffectiveStart(ZonedDateTime effectiveStart) { this.effectiveStart = effectiveStart; }
    public ZonedDateTime getEffectiveEnd() { return effectiveEnd; }
    public void setEffectiveEnd(ZonedDateTime effectiveEnd) { this.effectiveEnd = effectiveEnd; }
    public String getSourceText() { return sourceText; }
    public void setSourceText(String sourceText) { this.sourceText = sourceText; }
    public int getSequenceIndex() { return sequenceIndex; }
    public void setSequenceIndex(int sequenceIndex) { this.sequenceIndex = sequenceIndex; }
    public List<AltrvRouteEventEntity> getEvents() { return events; }
    public void setEvents(List<AltrvRouteEventEntity> events) { this.events = events; }
}
