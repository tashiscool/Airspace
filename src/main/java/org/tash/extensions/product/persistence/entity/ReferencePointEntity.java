package org.tash.extensions.product.persistence.entity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.Table;

@Entity
@Table(name = "ops_reference_point")
public class ReferencePointEntity extends BaseOperationalEntity {
    @Column(name = "identifier", nullable = false, unique = true)
    private String identifier;
    @Column(name = "point_type", nullable = false)
    private String pointType;
    @Column(name = "latitude", nullable = false)
    private double latitude;
    @Column(name = "longitude", nullable = false)
    private double longitude;
    @Column(name = "altitude_feet")
    private Double altitudeFeet;
    @Column(name = "source")
    private String source;
    @Column(name = "metadata_json", columnDefinition = "TEXT")
    private String metadataJson;

    public String getIdentifier() { return identifier; }
    public void setIdentifier(String identifier) { this.identifier = identifier; }
    public String getPointType() { return pointType; }
    public void setPointType(String pointType) { this.pointType = pointType; }
    public double getLatitude() { return latitude; }
    public void setLatitude(double latitude) { this.latitude = latitude; }
    public double getLongitude() { return longitude; }
    public void setLongitude(double longitude) { this.longitude = longitude; }
    public Double getAltitudeFeet() { return altitudeFeet; }
    public void setAltitudeFeet(Double altitudeFeet) { this.altitudeFeet = altitudeFeet; }
    public String getSource() { return source; }
    public void setSource(String source) { this.source = source; }
    public String getMetadataJson() { return metadataJson; }
    public void setMetadataJson(String metadataJson) { this.metadataJson = metadataJson; }
}
